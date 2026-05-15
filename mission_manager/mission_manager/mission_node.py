# mission_node.py — ROS2 node that owns and drives the mission state machine
#
# Architecture:
#   - rclpy.spin() runs in the main thread (handles all ROS callbacks)
#   - Mission state machine runs in a separate thread (_run_mission)
#   - This allows blocking mavros_utils calls (wait_for_altitude, etc.)
#     to use rclpy.spin_once safely without conflicting with the executor
#
# Usage:
#   ros2 run mission_manager mission_node

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from mavros_msgs.msg import State as MavrosState, RCIn
from sensor_msgs.msg import BatteryState

from mission_manager.state_machine import MissionSM
from mission_manager.config_loader import load_mission_config


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")

        # ----- Load config -----
        self.cfg = load_mission_config(self)

        # ----- State machine -----
        self.sm = MissionSM(ros_node=self, cfg=self.cfg)

        # ----- Publishers -----
        self.cmd_pub      = self.create_publisher(String, "/drone_command", 10)
        self.status_pub   = self.create_publisher(String, "/mission/status", 10)
        self.telemetry_pub = self.create_publisher(String, "/mission/telemetry", 10)

        # ----- Subscribers -----
        self.create_subscription(String, "/qr/detection", self._qr_detection_cb, 10)

        self.mavros_state = None
        self.create_subscription(MavrosState, "/mavros/state", self._mavros_state_cb, 10)

        # QoS fix: MAVROS publishes battery as BEST_EFFORT
        battery_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.battery_voltage = None
        self.create_subscription(BatteryState, "/mavros/battery", self._battery_cb, battery_qos)

        self._rc_channels = []
        self.create_subscription(RCIn, "/mavros/rc/in", self._rc_in_cb, 10)

        # ----- Internal flags -----
        self._battery_critical  = False
        self._rc_auto_active    = False
        self._abort_requested   = False   # set by battery/RC abort, checked in mission thread
        self._mission_complete  = False

        # ----- Telemetry timer (runs in main thread, safe) -----
        self.create_timer(0.5, self._publish_telemetry)

        # ----- Mission thread -----
        self._mission_thread = threading.Thread(
            target=self._run_mission, daemon=True
        )
        self._mission_thread.start()

        self.get_logger().info(
            "[mission_node] Initialized. Mission thread started."
        )

    # ===================================================================
    # Subscriber callbacks  (all run in main executor thread)
    # ===================================================================

    def _mavros_state_cb(self, msg: MavrosState):
        self.mavros_state = msg

    def _battery_cb(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        if msg.voltage > 0 and msg.voltage < self.cfg.battery_critical_v:
            if not self._battery_critical:
                self._battery_critical = True
                self.get_logger().error(
                    f"[mission_node] BATTERY CRITICAL: {msg.voltage:.1f}V "
                    f"< {self.cfg.battery_critical_v}V — requesting abort"
                )
                self._abort_requested = True

    def _rc_in_cb(self, msg: RCIn):
        self._rc_channels = list(msg.channels)
        ch     = self.cfg.rc_auto_channel
        thresh = self.cfg.rc_auto_pwm_thresh

        if ch < len(self._rc_channels):
            pwm      = self._rc_channels[ch]
            was_auto = self._rc_auto_active

            if pwm > thresh:
                self._rc_auto_active = True
            else:
                self._rc_auto_active = False
                if was_auto:
                    self.get_logger().warn(
                        f"[mission_node] RC ch{ch}={pwm} < {thresh} — "
                        f"pilot took control, requesting abort"
                    )
                    self._abort_requested = True

    def _qr_detection_cb(self, msg: String):
        if self.sm.current_state.id != "ARENA_SEARCH":
            return

        try:
            parts       = msg.data.split("|")
            qr_content  = parts[0]
            lat         = float(parts[1])
            lon         = float(parts[2])
        except (IndexError, ValueError):
            self.get_logger().warn(
                f"[mission_node] Malformed QR detection: '{msg.data}'"
            )
            return

        if self.sm.mission_target_qr and qr_content == self.sm.mission_target_qr:
            self.sm.target_gps = (lat, lon)
            self.get_logger().info(
                f"[mission_node] QR MATCHED: '{qr_content}' at ({lat}, {lon})"
            )
            self.sm.qr_matched()
        else:
            self.get_logger().debug(
                f"[mission_node] QR seen '{qr_content}' — not target "
                f"(want '{self.sm.mission_target_qr}')"
            )

    # ===================================================================
    # Mission thread  (blocking calls live here, NOT in callbacks)
    # ===================================================================

    def _run_mission(self):
        """
        Linear mission sequence running in its own thread.
        Each sm.transition() call triggers the matching on_enter in
        state_machine.py, which blocks until that phase is complete.
        The main thread keeps spinning so all ROS callbacks stay live.
        """
        try:
            # ── Wait for FCU ready ──────────────────────────────────────
            self.get_logger().info("[mission_node] Waiting for MAVROS connection...")
            while self.mavros_state is None or not self.mavros_state.connected:
                if not rclpy.ok():
                    return
                time.sleep(0.5)

            self.get_logger().info("[mission_node] MAVROS connected. Waiting for EKF...")
            while self.mavros_state.system_status != 3:
                if not rclpy.ok():
                    return
                time.sleep(0.5)

            self.get_logger().info("[mission_node] FCU ready (EKF healthy). Starting mission.")
            self._abort_requested = False
            self._battery_critical = False

            self.sm.auto_activate()

            # ── Phase 1: Scan reference QR ─────────────────────────────
            self._publish_status("SCAN_REFERENCE_QR")
            self.sm.auto_activate()          # IDLE → SCAN_REFERENCE_QR (on_enter blocks)
            if self._check_abort():
                return
            self.sm.qr_scanned()             # → TAKEOFF

            # ── Phase 2: Takeoff ────────────────────────────────────────
            self._publish_status("TAKEOFF")
            # on_enter_TAKEOFF already ran (triggered by qr_scanned transition)
            # just need to advance the state
            if self._check_abort():
                return
            self.sm.airborne()               # → CORRIDOR_1_TRANSIT

            # ── Phase 3: Corridor 1 (clean) ─────────────────────────────
            self._publish_status("CORRIDOR_1_TRANSIT")
            if self._check_abort():
                return
            self.sm.c1_complete()            # → ARENA_SEARCH

            # ── Phase 4: Arena search — wait for QR match ───────────────
            self._publish_status("ARENA_SEARCH")
            self.get_logger().info("[mission_node] Searching arena for QR target...")
            while self.sm.current_state.id == "ARENA_SEARCH":
                if self._check_abort():
                    return
                time.sleep(0.5)
            # _qr_detection_cb already called sm.qr_matched() → TARGET_FOUND

            # ── Phase 5: Navigate to drop ───────────────────────────────
            self._publish_status("TARGET_FOUND")
            if self._check_abort():
                return
            self.sm.nav_to_drop()            # → NAVIGATE_TO_DROP (on_enter blocks)

            self._publish_status("NAVIGATE_TO_DROP")
            if self._check_abort():
                return
            self.sm.drop_position_reached()  # → DROPPING (on_enter blocks)

            # ── Phase 6: Drop payload ───────────────────────────────────
            self._publish_status("DROPPING")
            if self._check_abort():
                return
            self.sm.drop_complete()          # → CORRIDOR_2_TRANSIT

            # ── Phase 7: Corridor 2 (obstacles) ────────────────────────
            self._publish_status("CORRIDOR_2_TRANSIT")
            if self._check_abort():
                return
            self.sm.c2_complete()            # → HOMING (on_enter blocks)

            # ── Phase 8: Home and land ──────────────────────────────────
            self._publish_status("HOMING")
            if self._check_abort():
                return
            self.sm.touchdown()              # → LANDED (on_enter disarms)

            # ── Done ────────────────────────────────────────────────────
            self._mission_complete = True
            self._publish_status("LANDED")
            self.get_logger().info("[mission_node] Mission complete!")

        except Exception as e:
            self.get_logger().error(f"[mission_node] Mission thread crashed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _check_abort(self) -> bool:
        """
        Check if an abort has been requested (battery, RC switch, or shutdown).
        If so, trigger sm.abort() and return True so the mission thread exits.
        """
        if not rclpy.ok():
            return True

        if self._abort_requested:
            current = self.sm.current_state.id
            if current not in ("IDLE", "RC_MODE", "LANDED"):
                self.get_logger().warn(
                    f"[mission_node] Abort triggered from state {current}"
                )
                self.sm.abort()
            return True

        return False

    # ===================================================================
    # Telemetry / status helpers  (called from main thread timer)
    # ===================================================================

    def _publish_status(self, state_id: str):
        msg = String()
        msg.data = state_id
        self.status_pub.publish(msg)

    def _publish_telemetry(self):
        state_id = self.sm.current_state.id
        self._publish_status(state_id)

        data = {
            "state":         state_id,
            "battery_v":     round(self.battery_voltage, 2) if self.battery_voltage else None,
            "target_qr":     self.sm.mission_target_qr,
            "target_gps":    list(self.sm.target_gps) if self.sm.target_gps else None,
            "home_gps":      list(self.sm.home_position) if self.sm.home_position else None,
            "drop_confirmed": self.sm.drop_confirmed,
            "rc_auto":       self._rc_auto_active,
            "fcu_connected": self.mavros_state.connected if self.mavros_state else False,
            "fcu_armed":     self.mavros_state.armed if self.mavros_state else False,
            "fcu_mode":      self.mavros_state.mode if self.mavros_state else "UNKNOWN",
            "abort_requested": self._abort_requested,
            "mission_complete": self._mission_complete,
        }
        msg = String()
        msg.data = json.dumps(data)
        self.telemetry_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)          # main thread: handles all ROS callbacks
    except KeyboardInterrupt:
        node.get_logger().info("[mission_node] Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()