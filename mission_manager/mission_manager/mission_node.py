# mission_node.py — ROS2 node that owns and drives the Mission 2 sequence
#
# Architecture:
#   - rclpy.spin() runs in the main thread. Every ROS callback here only
#     records data / sets flags and returns immediately — no service calls,
#     no state transitions that do work. (A blocking callback would freeze the
#     single-threaded executor, and with it every service response.)
#   - The mission runs linearly in a worker thread (_run_mission). It calls
#     services and sleep-polls their futures while the main thread spins.
#   - Abort conditions (RC switch low, battery critical, obstacle sensor dead)
#     are checked continuously by _tick() inside every wait, so the mission can
#     be stopped mid-leg, not only between phases.
#
# Usage:
#   ros2 run mission_manager mission_node

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import State as MavrosState, RCIn
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus

from mission_manager import mavros_utils as mav
from mission_manager import srv_helpers as srv
from mission_manager.config_loader import load_mission_config
from mission_manager.state_machine import MissionSM


class MissionAbort(Exception):
    """Raised inside the mission thread to leave the sequence.

    kind:
      'pilot'   — RC auto switch low: pilot takes over (LOITER).
      'battery' — battery critical: ArduPilot BATT_FS is primary; we only stop
                  commanding so we never fight the FC failsafe.
      'failure' — companion-detected failure (sensor dead, leg failed, ...):
                  LAND if airborne, disarm/stay down if on the ground.
    """

    def __init__(self, kind: str, reason: str):
        super().__init__(f"{kind}: {reason}")
        self.kind = kind
        self.reason = reason


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")

        self.cfg = load_mission_config(self)
        self.sm = MissionSM(logger=self.get_logger())

        # ----- Publishers -----
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.telemetry_pub = self.create_publisher(String, "/mission/telemetry", 10)
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Target ID for the perception Phase-5 matcher (latched so a late
        # subscriber still gets it).
        self.delivery_id_pub = self.create_publisher(String, "/qr_scanner/delivery_id", latched)

        # MAVROS publishes BEST_EFFORT; a RELIABLE subscriber gets nothing.
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ----- Live data (written by callbacks, read by the mission thread) -----
        self.mavros_state = None
        self.battery_voltage = None
        self.gps = None                 # (lat, lon) with a valid fix
        self.rel_alt = None             # metres above home
        self.home = None                # (lat, lon) recorded before arming
        self._rc_auto_active = False
        self._sensor_live = False       # coverage_guard defaults are fail-safe False
        self._coverage_ok = False
        self._sensor_bad_since = None

        self._abort = None              # (kind, reason) once requested
        self._airborne = False
        self._mission_complete = False
        self._qr_match = None           # (lat, lon) of the confirmed target QR

        self.create_subscription(MavrosState, "/mavros/state", self._mavros_state_cb, mavros_qos)
        self.create_subscription(BatteryState, "/mavros/battery", self._battery_cb, mavros_qos)
        self.create_subscription(RCIn, "/mavros/rc/in", self._rc_in_cb, mavros_qos)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._gps_cb, mavros_qos)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt", self._rel_alt_cb, mavros_qos)
        self.create_subscription(String, "/qr/detection", self._qr_detection_cb, 10)
        # coverage_guard publishes these RELIABLE at 10 Hz.
        self.create_subscription(Bool, "/avoidance/sensor_live", self._sensor_live_cb, 10)
        self.create_subscription(Bool, "/avoidance/coverage_ok", self._coverage_ok_cb, 10)

        self.create_timer(1.0 / self.cfg.tick_rate_hz, self._publish_telemetry)

        self._mission_thread = threading.Thread(target=self._run_mission, daemon=True)
        self._mission_thread.start()
        self.get_logger().info("[mission_node] Initialized. Mission thread started.")

    # ===================================================================
    # Subscriber callbacks — record + flag only, never block
    # ===================================================================

    def _mavros_state_cb(self, msg: MavrosState):
        self.mavros_state = msg

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.gps = (msg.latitude, msg.longitude)

    def _rel_alt_cb(self, msg: Float64):
        self.rel_alt = float(msg.data)

    def _battery_cb(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        if 0.0 < msg.voltage < self.cfg.battery_critical_v and self._airborne:
            self._request_abort(
                "battery",
                f"battery critical {msg.voltage:.2f} V < {self.cfg.battery_critical_v} V")

    def _rc_in_cb(self, msg: RCIn):
        idx = int(self.cfg.rc_auto_channel) - 1      # physical CHn -> index n-1
        if not 0 <= idx < len(msg.channels):
            return
        pwm = msg.channels[idx]
        was_auto = self._rc_auto_active
        self._rc_auto_active = pwm > self.cfg.rc_auto_pwm_thresh
        if was_auto and not self._rc_auto_active:
            self._request_abort(
                "pilot", f"RC CH{idx + 1}={pwm} < {self.cfg.rc_auto_pwm_thresh}")

    def _sensor_live_cb(self, msg: Bool):
        self._sensor_live = bool(msg.data)

    def _coverage_ok_cb(self, msg: Bool):
        self._coverage_ok = bool(msg.data)

    def _qr_detection_cb(self, msg: String):
        # Format from qr_scanner_node: "content|qr_lat|qr_lon", published only
        # after the confirmation gate fires. Only record it; the mission
        # thread acts on it.
        if self.sm.state_id != "ARENA_SEARCH" or self._qr_match is not None:
            return
        try:
            content, lat, lon = msg.data.split("|")
            lat, lon = float(lat), float(lon)
        except ValueError:
            self.get_logger().warn(f"[mission_node] Malformed QR detection: '{msg.data}'")
            return
        if content == self.sm.mission_target_qr:
            self.get_logger().info(
                f"[mission_node] QR MATCHED '{content}' at ({lat:.7f}, {lon:.7f})")
            self._qr_match = (lat, lon)
        else:
            self.get_logger().info(f"[mission_node] QR '{content}' seen — not the target")

    def _request_abort(self, kind: str, reason: str):
        if self._abort is None:
            self.get_logger().error(f"[mission_node] ABORT REQUESTED ({kind}): {reason}")
            self._abort = (kind, reason)

    # ===================================================================
    # Waiting helpers (mission thread only)
    # ===================================================================

    def _tick(self):
        """Raise MissionAbort if any abort condition holds. Call often."""
        if not rclpy.ok():
            raise MissionAbort("failure", "ROS shutting down")
        if self._abort is not None:
            raise MissionAbort(*self._abort)
        if self._airborne and self.cfg.require_avoidance_sensor:
            if self._sensor_live:
                self._sensor_bad_since = None
            else:
                now = time.time()
                if self._sensor_bad_since is None:
                    self._sensor_bad_since = now
                    self.get_logger().warn("[mission_node] Obstacle sensor NOT live")
                elif now - self._sensor_bad_since > self.cfg.sensor_fail_timeout_s:
                    raise MissionAbort(
                        "failure",
                        f"obstacle sensor dead > {self.cfg.sensor_fail_timeout_s:.0f}s")

    def _sleep(self, secs: float):
        end = time.time() + secs
        while time.time() < end:
            self._tick()
            time.sleep(0.1)

    def _wait_for(self, cond, timeout: float, what: str) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._tick()
            if cond():
                return True
            time.sleep(0.1)
        self.get_logger().error(f"[mission_node] Timeout waiting for {what} ({timeout:.0f}s)")
        return False

    def _should_abort(self) -> bool:
        try:
            self._tick()
            return False
        except MissionAbort:
            return True

    def _armed(self) -> bool:
        return bool(self.mavros_state is not None and self.mavros_state.armed)

    def _goto_and_wait(self, lat, lon, alt, tolerance, timeout, face_target=True) -> bool:
        """GUIDED goto with arrival check. Holds while the obstacle sensor is
        not live and re-sends the target once it recovers (the 10 s sensor
        timeout in _tick() turns a long outage into an abort)."""
        def send():
            yaw = None
            if face_target and self.gps is not None:
                yaw = mav.yaw_enu_towards(self.gps[0], self.gps[1], lat, lon)
            mav.goto(self, lat, lon, alt, publish_count=self.cfg.goto_publish_count, yaw_enu=yaw)

        send()
        holding = False
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._tick()
            if self.cfg.require_avoidance_sensor and not self._sensor_live:
                if not holding and self.gps is not None:
                    self.get_logger().warn("[mission_node] Sensor not live — holding position")
                    mav.goto(self, self.gps[0], self.gps[1], alt, publish_count=5)
                    holding = True
                    deadline += self.cfg.sensor_fail_timeout_s
            elif holding:
                self.get_logger().info("[mission_node] Sensor recovered — resuming goto")
                holding = False
                send()
            elif self.gps is not None and mav.haversine(*self.gps, lat, lon) <= tolerance:
                return True
            time.sleep(0.2)
        self.get_logger().error(f"[mission_node] Arrival timeout at ({lat:.7f}, {lon:.7f})")
        return False

    def _call_leg(self, srv_name: str, timeout: float):
        """Call a blocking path_planner leg service; cancels it on abort."""
        try:
            return srv.call_trigger(self, srv_name, call_timeout=timeout,
                                    should_abort=self._should_abort)
        except srv.CallAborted:
            self._stop_planner()
            self._tick()          # re-raise the MissionAbort that caused it
            raise

    def _stop_planner(self):
        try:
            srv.call_trigger(self, srv.SRV_PLANNER_STOP, call_timeout=3.0, wait_timeout=2.0)
        except Exception as e:      # best effort
            self.get_logger().warn(f"[mission_node] planner stop failed: {e}")

    # ===================================================================
    # Mission phases
    # ===================================================================

    def _wait_for_start(self):
        self.get_logger().info("[mission_node] Waiting for MAVROS connection...")
        while self.mavros_state is None or not self.mavros_state.connected:
            self._tick()
            time.sleep(0.5)

        # ArduPilot only streams what is requested (SITL --no-mavproxy) or
        # what SRx_ params enable (hardware). Ask explicitly for what we use.
        mav.request_message_intervals(self, rate_hz=self.cfg.stream_rate_hz)

        if self.cfg.require_rc_switch:
            self.get_logger().info(
                f"[mission_node] Waiting for pilot: RC CH{self.cfg.rc_auto_channel} "
                f"> {self.cfg.rc_auto_pwm_thresh} to start the mission")
            while not self._rc_auto_active:
                self._tick()
                time.sleep(0.2)
            self.get_logger().info("[mission_node] RC auto switch ON — starting mission")

    def _scan_reference_qr(self) -> str:
        """Phase 1 (on the ground): read the reference QR, with retries."""
        for attempt in range(1, int(self.cfg.qr_scan_attempts) + 1):
            self.get_logger().info(
                f"[mission_node] Reference QR scan attempt {attempt}/{self.cfg.qr_scan_attempts}")
            res = srv.call_trigger(
                self, srv.SRV_SCAN_QR,
                call_timeout=self.cfg.qr_scan_timeout_s + 10.0,   # > scanner's own timeout
                should_abort=self._should_abort)
            if res is not None and res.success and res.message:
                return res.message
            self._sleep(1.0)

        if self.cfg.allow_qr_fallback:
            self.get_logger().warn(
                f"[mission_node] QR scan failed — using fallback ID '{self.cfg.qr_fallback_id}' "
                "(allow_qr_fallback=true, SITL only)")
            return self.cfg.qr_fallback_id
        raise MissionAbort("failure", "reference QR could not be read — staying on the ground")

    def _preflight(self):
        if not self._wait_for(lambda: self.gps is not None, 60.0, "GPS fix"):
            raise MissionAbort("failure", "no GPS fix")
        self.home = self.gps
        self.get_logger().info(f"[mission_node] Home: ({self.home[0]:.7f}, {self.home[1]:.7f})")
        if self.cfg.require_avoidance_sensor and not self._wait_for(
                lambda: self._sensor_live, self.cfg.sensor_preflight_timeout_s,
                "obstacle sensor live (/avoidance/sensor_live)"):
            raise MissionAbort("failure", "obstacle sensor not live before takeoff")

    def _takeoff(self):
        alt = self.cfg.cruise_altitude_m
        if not mav.set_mode(self, "GUIDED"):
            raise MissionAbort("failure", "could not set GUIDED")
        # A cold FC rejects arming for ~30-60 s while the EKF settles: retry.
        deadline = time.time() + self.cfg.arm_timeout_s
        while not self._armed():
            self._tick()
            if time.time() > deadline:
                raise MissionAbort("failure", "arming timed out (check pre-arm messages)")
            if mav.arm(self):
                self._wait_for(self._armed, 5.0, "armed state")
            if not self._armed():
                self._sleep(2.0)

        if not mav.takeoff(self, alt):
            raise MissionAbort("failure", "takeoff command rejected")
        self._airborne = True
        if not self._wait_for(
                lambda: self.rel_alt is not None
                and abs(self.rel_alt - alt) <= self.cfg.altitude_tolerance_m,
                self.cfg.altitude_timeout_s, f"altitude {alt} m"):
            raise MissionAbort("failure", "did not reach takeoff altitude")
        self._sleep(1.0)

    def _fly_leg(self, srv_name: str, timeout: float, label: str):
        res = self._call_leg(srv_name, timeout)
        if res is None:
            raise MissionAbort("failure", f"{label}: no response from path_planner")
        if not res.success:
            raise MissionAbort("failure", f"{label} failed: {res.message}")

    def _arena_search(self):
        """Run the lawnmower until the target QR is confirmed or the pattern ends.
        Returns the QR (lat, lon) or None."""
        self._qr_match = None
        self.delivery_id_pub.publish(String(data=self.sm.mission_target_qr))
        srv.call_trigger(self, srv.SRV_START_QR_SCAN, call_timeout=self.cfg.service_timeout_s)

        result = {}

        def worker():
            try:
                result["res"] = srv.call_trigger(
                    self, srv.SRV_RUN_LAWNMOWER, call_timeout=self.cfg.lawnmower_timeout_s + 30.0)
            except Exception as e:
                result["err"] = e

        t = threading.Thread(target=worker, daemon=True)
        t.start()
        try:
            while t.is_alive() and self._qr_match is None:
                self._tick()
                time.sleep(0.2)
        except MissionAbort:
            self._stop_planner()
            raise

        if self._qr_match is not None:
            self._stop_planner()           # ends the lawnmower call
            t.join(timeout=5.0)
            return self._qr_match

        res = result.get("res")
        if res is not None and not res.success and res.message.startswith("SENSOR"):
            raise MissionAbort("failure", f"arena search: {res.message}")
        # Pattern finished: give a confirmation that is mid-gate a moment.
        self._wait_for(lambda: self._qr_match is not None, 2.0, "late QR confirmation")
        return self._qr_match

    def _drop_payload(self, qr_lat, qr_lon):
        alt = self.cfg.drop_altitude_m
        if not self._goto_and_wait(qr_lat, qr_lon, alt, self.cfg.drop_arrival_tolerance_m,
                                   self.cfg.drop_arrival_timeout_s, face_target=False):
            self.get_logger().warn("[mission_node] Not settled over the QR — dropping anyway")
        self.sm.drop_position_reached()
        self._sleep(self.cfg.drop_settle_s)
        res = srv.call_trigger(self, srv.SRV_PAYLOAD_RELEASE,
                               call_timeout=self.cfg.service_timeout_s)
        self.sm.drop_confirmed = bool(res is not None and res.success)
        if not self.sm.drop_confirmed:
            self.get_logger().error("[mission_node] Payload release NOT confirmed")

    def _land_and_wait(self):
        if not mav.land(self):
            mav.land(self)          # one retry; LAND is the end of the line
        if not self._wait_for(lambda: not self._armed(), self.cfg.land_timeout_s,
                              "touchdown + auto-disarm"):
            raise MissionAbort("failure", "landing not confirmed")
        self._airborne = False

    def _run_mission(self):
        try:
            self._wait_for_start()

            self.sm.auto_activate()
            self.sm.mission_target_qr = self._scan_reference_qr()
            self.get_logger().info(f"[mission_node] Target locked: '{self.sm.mission_target_qr}'")
            self._preflight()

            self.sm.qr_scanned()
            self._takeoff()

            self.sm.airborne()
            self._fly_leg(srv.SRV_FLY_CORRIDOR_1, self.cfg.corridor_timeout_s + 30.0, "corridor 1")

            self.sm.c1_complete()
            target = self._arena_search()
            if target is not None:
                self.sm.qr_matched()
                self._drop_payload(*target)
                self.sm.drop_complete()
            else:
                self.get_logger().warn(
                    "[mission_node] Target QR not found — keeping payload, continuing")
                self.sm.search_exhausted()

            self._fly_leg(srv.SRV_FLY_CORRIDOR_2, self.cfg.corridor_timeout_s + 30.0, "corridor 2")

            self.sm.c2_complete()
            if not self._goto_and_wait(*self.home, self.cfg.cruise_altitude_m,
                                       self.cfg.arrival_tolerance_m, self.cfg.arrival_timeout_s):
                self.get_logger().warn("[mission_node] Home not reached in time — landing here")

            self.sm.home_reached()
            self._land_and_wait()
            self.sm.touchdown()
            self._mission_complete = True
            self.get_logger().info("[mission_node] Mission complete!")

        except MissionAbort as a:
            self._handle_abort(a)
        except Exception as e:
            import traceback
            self.get_logger().error(f"[mission_node] Mission thread crashed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._handle_abort(MissionAbort("failure", f"crash: {e}"))

    def _handle_abort(self, a: MissionAbort):
        self.get_logger().error(f"[mission_node] MISSION ABORTED — {a}")
        self._stop_planner()
        try:
            if a.kind == "pilot":
                if self.sm.state_id not in ("LANDED", "RC_MODE", "ABORTED"):
                    self.sm.abort()
                if self._armed():
                    mav.set_mode(self, "LOITER")     # pilot flies from here
            else:
                if self.sm.state_id not in ("LANDED", "RC_MODE", "ABORTED"):
                    self.sm.failsafe()
                if a.kind == "battery":
                    self.get_logger().error(
                        "[mission_node] Leaving control to ArduPilot battery failsafe "
                        "(BATT_FS_CRT_ACT). Make sure it is configured.")
                elif self._armed() and self._airborne:
                    mav.land(self)
                elif self._armed():
                    mav.disarm(self)
        except Exception as e:
            self.get_logger().error(f"[mission_node] abort handling error: {e}")

    # ===================================================================
    # Telemetry (main thread timer)
    # ===================================================================

    def _publish_telemetry(self):
        state_id = self.sm.state_id
        self.status_pub.publish(String(data=state_id))
        st = self.mavros_state
        data = {
            "state": state_id,
            "battery_v": round(self.battery_voltage, 2) if self.battery_voltage else None,
            "rel_alt": round(self.rel_alt, 2) if self.rel_alt is not None else None,
            "target_qr": self.sm.mission_target_qr,
            "drop_confirmed": self.sm.drop_confirmed,
            "rc_auto": self._rc_auto_active,
            "sensor_live": self._sensor_live,
            "coverage_ok": self._coverage_ok,
            "fcu_connected": st.connected if st else False,
            "fcu_armed": st.armed if st else False,
            "fcu_mode": st.mode if st else "UNKNOWN",
            "abort": None if self._abort is None else f"{self._abort[0]}: {self._abort[1]}",
            "mission_complete": self._mission_complete,
        }
        self.telemetry_pub.publish(String(data=json.dumps(data)))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[mission_node] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
