# path_planner_node.py — ROS2 node that flies the mission legs via MAVROS
#
# Trigger services consumed by mission_manager (each blocks until the leg is
# done, cancelled, or failed):
#   /path_planner/fly_corridor_1   — corridor 1 (clean)
#   /path_planner/fly_corridor_2   — corridor 2 (static obstacles)
#   /path_planner/run_lawnmower    — BCD lawnmower search (arena_speed_ms)
#   /path_planner/stop             — cancel the active leg and hold position
#
# Response message is a machine-readable token first:
#   OK | CANCELLED | BUSY | NO_GPS | NO_CORRIDOR | BLOCKED | WP_TIMEOUT ... |
#   SENSOR_TIMEOUT ... | LEG_TIMEOUT ...
#
# Navigation: GUIDED GlobalPositionTarget setpoints (mavros_utils.goto) with
# the plain position controller (FC: GUID_OPTIONS bit 6 clear, OA_TYPE 0), so
# speed follows MAV_CMD_DO_CHANGE_SPEED. Each leg first sends a hold target,
# then sets the speed (entering a GUIDED submode resets the speed limit).
#
# CORRIDORS (1.524 m = 5 ft wide). Both corridors are flown the same way:
#   1. transit at cruise altitude to the corridor's first waypoint (on the
#      centreline, a few metres before the mouth), routed around red zones
#   2. face along the corridor and change to corridor_altitude_m there
#   3. wait until /corridor/perception sees both walls (else NO_CORRIDOR)
#   4. follow: at corridor_rate_hz send a target lookahead_m ahead along the
#      measured corridor direction, shifted across to the planned lateral
#      position (centre, or the free side of an obstacle) and at the planned
#      altitude (nominal, or over an obstacle). Logic: corridor_follow.py.
#   5. exit, whichever comes first:
#        - blue ground marker (/corridor/ground_marker) past half the corridor
#          length, then exit_clearance_m more
#        - the walls' end, measured ahead by perception: in the last ~2 m too
#          little wall is in view to fit, so the vehicle flies straight out
#          along its heading — only if the last valid plan was already clear
#          to advance (no obstacle, lined up beside one, or at climb altitude
#          over one), keeping that plan's altitude until the obstacle is
#          passed — and finishes exit_clearance_m past the measured end
#        - the full configured length flown and no walls in view
#        - hard stop at length + max_overrun_m
#   6. climb vertically to cruise altitude where the leg ended.
# The green ground marker (entrance) is logged as a cross-check.
# BendyRuler is NOT used: at 5 ft it cannot pass a 30-40 % obstacle (see
# corridor_follow.py header).
#
# Sensor gating:
#   lawnmower/transit: sensor_live False -> hold; resume when live again
#   corridor: no valid corridor fit (or sensor_live False) -> hold; after
#             coverage_backoff_after_s back off coverage_backoff_m along the
#             corridor; resume when it recovers
#   unhealthy > sensor_fail_timeout_s -> fail the leg with SENSOR_TIMEOUT
#   (mission_node then LANDs). An obstacle that can be neither passed nor
#   climbed holds the vehicle; after sensor_fail_timeout_s -> BLOCKED.
#
# Usage:
#   ros2 run path_planner path_planner_node

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from std_srvs.srv import Trigger
from std_msgs.msg import Bool, Float32MultiArray, Float64, String
from sensor_msgs.msg import NavSatFix, NavSatStatus

from mission_common import corridor_msg
from mission_common import mavros_utils as mav
from mission_common.config_loader import load_mission_config
from path_planner.corridor_follow import (
    CorridorTracker,
    FollowParams,
    ObstacleMemory,
    bounded_axis,
    gps_measurement,
    plan,
)
from path_planner.waypoint_gen import (
    build_transit_space,
    generate_mission_waypoints,
    gps_to_local,
    local_to_gps,
    plan_path,
)

_DOUBLE_ARRAY = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")
        self.cfg = load_mission_config(self)

        # ----- Course geometry (local ENU metres about origin) -----
        # PLACEHOLDERS — set the real corridors on competition day (yaml).
        self._corridor1_local = self._xy_param(
            "corridor1_waypoints", [-3.0, 13.0], [15.0, 15.0])
        self._corridor2_local = self._xy_param(
            "corridor2_waypoints", [13.0, -3.0], [-5.0, -5.0])

        defaults = {
            1: ([24.0, 36.0, 36.0, 24.0], [12.0, 12.0, 22.0, 22.0]),
            2: ([10.0, 15.0, 12.5], [5.0, 5.0, 10.0]),
            3: ([5.0, 10.0, 10.0, 7.0, 7.0, 5.0], [20.0, 20.0, 22.0, 22.0, 25.0, 25.0]),
        }
        self._red_zones = []
        for i, (dx, dy) in defaults.items():
            zone = self._xy_param(f"red_zone_{i}", dx, dy, suffix_x="_x", suffix_y="_y")
            if len(zone) >= 3:
                self._red_zones.append(zone)

        self.declare_parameter("arena_edge_margin_m", 1.0)
        edge_margin = float(self.get_parameter("arena_edge_margin_m").value)
        self._declare_corridor_params()

        # ----- Lawnmower (routed around red zones) -----
        wps = generate_mission_waypoints(
            origin_lat=self.cfg.origin_lat,
            origin_lon=self.cfg.origin_lon,
            arena_width=self.cfg.arena_width_m,
            arena_height=self.cfg.arena_height_m,
            altitude=self.cfg.cruise_altitude_m,
            hfov_deg=self.cfg.down_cam_hfov_deg,
            overlap=self.cfg.strip_overlap,
            red_zones_vertices=self._red_zones,
            red_zone_buffer=self.cfg.red_zone_buffer_m,
            entry_x=self._corridor1_local[-1][0],
            entry_y=self._corridor1_local[-1][1],
            speed_ms=self.cfg.arena_speed_ms,
            edge_margin=edge_margin,
        )
        self._lawnmower_local = wps["local_waypoints"]
        self.get_logger().info(
            f"[path_planner] {len(self._lawnmower_local)} lawnmower waypoints, "
            f"strip={wps['strip_width']:.1f} m, est. {wps['estimated_dist']:.0f} m / "
            f"{wps['estimated_time'] / 60:.1f} min")

        # Free space for transit hops (corridor ends, drop point -> corridor 2).
        self._transit_space = build_transit_space(
            self._corridor1_local + self._corridor2_local + self._lawnmower_local,
            self._red_zones, self.cfg.red_zone_buffer_m)

        # ----- Live data -----
        self._gps = None
        self._rel_alt = None
        self._yaw_enu = None            # rad, from compass heading
        self._sensor_live = False
        self._perception = None         # (dict, arrival time) newest, unconsumed
        self._marker = "none"

        # ----- Run control -----
        self._run_lock = threading.Lock()       # one leg at a time
        self._cancel = threading.Event()
        self._leg_deadline = 0.0

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._gps_cb, mavros_qos)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt", self._alt_cb, mavros_qos)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg", self._hdg_cb, mavros_qos)
        self.create_subscription(Bool, "/avoidance/sensor_live", self._live_cb, 10)
        self.create_subscription(Float32MultiArray, "/corridor/perception", self._perception_cb, 10)
        self.create_subscription(String, "/corridor/ground_marker", self._marker_cb, 10)

        # Leg services block for the whole leg: reentrant group + multithreaded
        # executor so /stop and the subscriptions keep running meanwhile.
        cb = ReentrantCallbackGroup()
        self.create_service(Trigger, "/path_planner/fly_corridor_1",
                            self._handle_corridor_1, callback_group=cb)
        self.create_service(Trigger, "/path_planner/fly_corridor_2",
                            self._handle_corridor_2, callback_group=cb)
        self.create_service(Trigger, "/path_planner/run_lawnmower",
                            self._handle_run_lawnmower, callback_group=cb)
        self.create_service(Trigger, "/path_planner/stop",
                            self._handle_stop, callback_group=cb)

        self.get_logger().info(
            "[path_planner] Ready — fly_corridor_1, fly_corridor_2, run_lawnmower, stop")

    # ===================================================================
    # Params / callbacks
    # ===================================================================

    def _xy_param(self, name, default_x, default_y, suffix_x="_x", suffix_y="_y"):
        nx, ny = name + suffix_x, name + suffix_y
        for n, d in ((nx, default_x), (ny, default_y)):
            if not self.has_parameter(n):
                self.declare_parameter(n, [float(v) for v in d], _DOUBLE_ARRAY)
        xs = list(self.get_parameter(nx).value)
        ys = list(self.get_parameter(ny).value)
        if len(xs) != len(ys):
            raise ValueError(f"{nx} and {ny} must have the same length")
        return list(zip(xs, ys))

    def _declare_corridor_params(self):
        d = FollowParams()
        decl = {
            "corridor_altitude_m": 1.5,
            "corridor_follow_speed_ms": 0.5,
            "corridor_rate_hz": 5.0,
            "corridor_entry_timeout_s": 15.0,
            "corridor_exit_clearance_m": 1.0,
            "corridor_max_overrun_m": 3.0,
            "corridor_exit_run_m": 3.0,         # max blind run to a measured wall end
            "corridor_exit_window_m": 3.0,      # a wall end must be this near the surveyed end
            "require_corridor_perception": True,
            "corridor_width_m": 1.524,          # GPS-only fallback (SITL) only
            "drone_width_m": d.drone_width_m,
            "side_clearance_m": d.side_clearance_m,
            "corridor_lookahead_m": d.lookahead_m,
            "obstacle_stop_dist_m": d.stop_dist_m,
            "obstacle_length_m": d.obstacle_length_m,
            "climb_clearance_m": d.climb_clearance_m,
            "body_below_camera_m": d.body_below_camera_m,
            "corridor_max_altitude_m": d.max_altitude_m,
        }
        for name, default in decl.items():
            self.declare_parameter(name, default)
        gp = lambda n: self.get_parameter(n).value          # noqa: E731
        self._follow = FollowParams(
            drone_width_m=float(gp("drone_width_m")),
            side_clearance_m=float(gp("side_clearance_m")),
            lookahead_m=float(gp("corridor_lookahead_m")),
            stop_dist_m=float(gp("obstacle_stop_dist_m")),
            obstacle_length_m=float(gp("obstacle_length_m")),
            climb_clearance_m=float(gp("climb_clearance_m")),
            body_below_camera_m=float(gp("body_below_camera_m")),
            max_altitude_m=float(gp("corridor_max_altitude_m")),
        )
        self._c_alt = float(gp("corridor_altitude_m"))
        self._c_speed = float(gp("corridor_follow_speed_ms"))
        self._c_period = 1.0 / float(gp("corridor_rate_hz"))
        self._c_entry_timeout = float(gp("corridor_entry_timeout_s"))
        self._c_exit_clear = float(gp("corridor_exit_clearance_m"))
        self._c_overrun = float(gp("corridor_max_overrun_m"))
        self._c_exit_run_m = float(gp("corridor_exit_run_m"))
        self._c_exit_window_m = float(gp("corridor_exit_window_m"))
        self._c_need_perception = bool(gp("require_corridor_perception"))
        self._c_width = float(gp("corridor_width_m"))
        if self._c_alt >= self._follow.max_altitude_m:
            raise ValueError("corridor_altitude_m must be below corridor_max_altitude_m")
        self.get_logger().info(
            f"[path_planner] corridor: alt {self._c_alt:.2f} m (max "
            f"{self._follow.max_altitude_m:.2f}), {self._c_speed:.2f} m/s, drone "
            f"width {self._follow.drone_width_m:.2f} m + {self._follow.side_clearance_m:.2f} m "
            f"clearance, perception {'REQUIRED' if self._c_need_perception else 'OFF (GPS centreline)'}")

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self._gps = (msg.latitude, msg.longitude)

    def _alt_cb(self, msg: Float64):
        self._rel_alt = float(msg.data)

    def _hdg_cb(self, msg: Float64):
        # compass_hdg: degrees clockwise from North -> ENU yaw (CCW from East)
        self._yaw_enu = _wrap(math.radians(90.0 - float(msg.data)))

    def _live_cb(self, msg: Bool):
        self._sensor_live = bool(msg.data)

    def _perception_cb(self, msg: Float32MultiArray):
        m = corridor_msg.decode(msg)
        if m is None:
            self.get_logger().error(
                "[path_planner] /corridor/perception layout mismatch — rebuild "
                "obstacle_avoidance and path_planner together", throttle_duration_sec=5.0)
            return
        self._perception = (m, time.time())

    def _marker_cb(self, msg: String):
        self._marker = msg.data

    # ===================================================================
    # Shared helpers
    # ===================================================================

    def _to_gps(self, xy):
        return local_to_gps(xy[0], xy[1], self.cfg.origin_lat, self.cfg.origin_lon)

    def _to_local(self, latlon):
        return gps_to_local(latlon[0], latlon[1], self.cfg.origin_lat, self.cfg.origin_lon)

    def _sensor_ok(self):
        return not self.cfg.require_avoidance_sensor or self._sensor_live

    def _hold(self, alt, yaw=None):
        if self._gps is not None:
            mav.goto(self, self._gps[0], self._gps[1], alt, publish_count=5, yaw_enu=yaw)

    def _set_speed(self, speed, alt):
        self._hold(alt)                 # enter the submode (resets speed limits)...
        mav.change_speed(self, speed)   # ...then set the speed

    def _wait_alt(self, alt, timeout, tol=None):
        tol = self.cfg.altitude_tolerance_m if tol is None else tol
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._cancel.is_set() or not rclpy.ok():
                return False
            if self._rel_alt is not None and abs(self._rel_alt - alt) <= tol:
                return True
            time.sleep(0.2)
        return False

    def _fly_to(self, latlon, alt, speed, leg):
        """Fly one waypoint, nose along the direction of travel.
        Returns 'OK', 'CANCELLED', 'WP_TIMEOUT', 'SENSOR_TIMEOUT', 'LEG_TIMEOUT'."""
        dist = mav.haversine(*self._gps, *latlon)
        wp_deadline = time.time() + 10.0 + 2.5 * dist / max(speed, 0.1)

        def send():
            yaw = mav.yaw_enu_towards(self._gps[0], self._gps[1], latlon[0], latlon[1])
            mav.goto(self, latlon[0], latlon[1], alt,
                     publish_count=self.cfg.goto_publish_count, yaw_enu=yaw)

        send()
        bad_since = None
        while True:
            if self._cancel.is_set() or not rclpy.ok():
                return "CANCELLED"
            now = time.time()
            if not self._sensor_ok():
                if bad_since is None:
                    bad_since = now
                    self.get_logger().warn(f"[path_planner] {leg}: obstacle sensor not live — holding")
                    self._hold(alt)
                if now - bad_since > self.cfg.sensor_fail_timeout_s:
                    return "SENSOR_TIMEOUT"
                # Pausing must not eat the waypoint / leg budget.
                wp_deadline += 0.2
                self._leg_deadline += 0.2
                time.sleep(0.2)
                continue
            if bad_since is not None:
                self.get_logger().info(f"[path_planner] {leg}: sensor OK — resuming")
                bad_since = None
                send()

            if mav.haversine(*self._gps, *latlon) <= self.cfg.mavros_arrival_tolerance_m:
                return "OK"
            if now > self._leg_deadline:
                return "LEG_TIMEOUT"
            if now > wp_deadline:
                return "WP_TIMEOUT"
            time.sleep(0.2)

    def _transit(self, leg, target_local, alt, speed, skip_timed_out=False, extra=()):
        """Route from here to target_local (+ extra waypoints) around red zones."""
        start = self._to_local(self._gps)
        approach = plan_path(start, target_local, self._transit_space)[1:-1]
        route = [self._to_gps(p) for p in approach + [target_local] + list(extra)]
        for i, wp in enumerate(route):
            self.get_logger().info(
                f"[path_planner] {leg} WP{i + 1}/{len(route)}: ({wp[0]:.7f}, {wp[1]:.7f})")
            result = self._fly_to(wp, alt, speed, leg)
            if result == "WP_TIMEOUT" and skip_timed_out:
                self.get_logger().warn(f"[path_planner] {leg}: WP{i + 1} timeout, skipping")
                continue
            if result != "OK":
                return f"{result} {leg} at WP{i + 1}/{len(route)}"
        return None

    def _run(self, leg, timeout, body):
        """One leg at a time, with a deadline; body() returns (ok, message)."""
        if not self._run_lock.acquire(blocking=False):
            return False, "BUSY another leg is active"
        try:
            self._cancel.clear()
            if self._gps is None:
                return False, "NO_GPS"
            self._leg_deadline = time.time() + timeout
            return body()
        except Exception as e:      # a bug must fail the leg, not kill the node
            import traceback
            self.get_logger().error(f"[path_planner] {leg} crashed:\n{traceback.format_exc()}")
            self._hold(self._rel_alt if self._rel_alt is not None else self.cfg.cruise_altitude_m)
            return False, f"ERROR {leg}: {type(e).__name__}: {e}"
        finally:
            self._run_lock.release()

    # ===================================================================
    # Lawnmower
    # ===================================================================

    def _lawnmower(self):
        alt = self.cfg.cruise_altitude_m
        self._set_speed(self.cfg.arena_speed_ms, alt)
        wps = self._lawnmower_local
        err = self._transit("lawnmower", wps[0], alt, self.cfg.arena_speed_ms,
                            skip_timed_out=True, extra=wps[1:])
        return (False, err) if err else (True, "OK lawnmower complete")

    # ===================================================================
    # Corridor
    # ===================================================================

    def _corridor(self, leg, local_wps):
        cruise = self.cfg.cruise_altitude_m
        entry, end = self._to_gps(local_wps[0]), self._to_gps(local_wps[-1])
        d_n, d_e = mav.ne_offset(entry[0], entry[1], end[0], end[1])
        length = math.hypot(d_n, d_e)
        if length < 2.0:
            return False, f"CONFIG {leg}: first and last waypoint {length:.1f} m apart"
        axis = (d_n / length, d_e / length)
        axis_yaw = math.atan2(d_n, d_e)

        # 1. transit at cruise altitude to the point before the mouth
        self._set_speed(self.cfg.transit_speed_ms, cruise)
        err = self._transit(leg, local_wps[0], cruise, self.cfg.transit_speed_ms)
        if err:
            return False, err

        # 2. face along the corridor, change to corridor altitude
        self.get_logger().info(
            f"[path_planner] {leg}: at entry, {length:.1f} m corridor heading "
            f"{math.degrees(axis_yaw):.0f} deg ENU — to {self._c_alt:.2f} m")
        mav.goto(self, entry[0], entry[1], self._c_alt, publish_count=10, yaw_enu=axis_yaw)
        if not self._wait_alt(self._c_alt, 30.0, tol=2 * self._follow.alt_tol_m):
            return False, f"{'CANCELLED' if self._cancel.is_set() else 'WP_TIMEOUT'} {leg} corridor altitude"
        mav.change_speed(self, self._c_speed)

        # 3-5. follow
        result = self._follow_corridor(leg, entry, axis, axis_yaw, length)

        # 6. climb straight up where we are (never diagonally across a wall)
        if not result[1].startswith("CANCELLED"):
            self._hold(cruise)
            self._wait_alt(cruise, 30.0)
        return result

    def _progress(self, entry, axis):
        """(along, left) metres of the vehicle relative to the entry point."""
        d_n, d_e = mav.ne_offset(entry[0], entry[1], self._gps[0], self._gps[1])
        return d_n * axis[0] + d_e * axis[1], d_n * axis[1] - d_e * axis[0]

    def _follow_corridor(self, leg, entry, axis, axis_yaw, length):
        fp = self._follow
        track, memory = CorridorTracker(fp), ObstacleMemory(fp)
        nominal = self._c_alt
        started = False
        wait_until = time.time() + self._c_entry_timeout
        entrance_seen = False
        exit_s = None
        walls_end_s = None          # progress where the walls were seen to end
        mouth_s = float("nan")      # progress of the mouth, while it is ahead
        bad_since = None
        backed_off = False
        blocked_since = None
        last_mode = None
        last_cmd = None             # plan from the last valid frame
        self._perception = None

        while True:
            t0 = time.time()
            if self._cancel.is_set() or not rclpy.ok():
                return False, f"CANCELLED {leg}"
            if t0 > self._leg_deadline:
                return False, f"LEG_TIMEOUT {leg}"
            s, lateral = self._progress(entry, axis)
            alt = self._rel_alt if self._rel_alt is not None else nominal
            yaw = self._yaw_enu if self._yaw_enu is not None else axis_yaw

            # --- measurements -> tracker / obstacle memory ---
            if self._c_need_perception:
                pm, self._perception = self._perception, None
                cam_yaw = yaw if pm is None or math.isnan(pm[0]["cam_yaw"]) else pm[0]["cam_yaw"]
                if pm is not None and track.update(pm[0], pm[1], cam_yaw):
                    m = pm[0]
                    cam_alt = alt if math.isnan(m["cam_alt"]) else m["cam_alt"]
                    if not math.isnan(m["mouth_dist"]):
                        mouth_s = s + m["mouth_dist"]
                    # Only a wall end near the surveyed end can be the exit.
                    if not math.isnan(m["walls_end"]) and \
                            s + m["walls_end"] >= length - self._c_exit_window_m:
                        walls_end_s = s + m["walls_end"]
                    if not math.isnan(m["obs_dist"]):
                        memory.update(m, track.centre, s, cam_alt)
                    else:
                        memory.unseen(s, cam_alt, m["slice_lo"],
                                      climbing=alt > nominal + fp.alt_tol_m)
            else:
                track.update(gps_measurement(lateral, _wrap(axis_yaw - yaw), self._c_width),
                             t0, yaw)
            c_axis = bounded_axis(track.axis, axis_yaw, fp.max_axis_dev_deg)
            if memory.passed(s):
                self.get_logger().info(f"[path_planner] {leg}: obstacle passed at s={s:.1f} m")

            # --- entrance / exit ---
            if self._marker == "green" and not entrance_seen:
                entrance_seen = True
                self.get_logger().info(f"[path_planner] {leg}: ENTRANCE marker (green) at s={s:.1f} m")
            if self._marker == "blue" and exit_s is None and s > 0.5 * length:
                exit_s = s
                self.get_logger().info(f"[path_planner] {leg}: EXIT marker (blue) at s={s:.1f} m")
            if exit_s is not None and s >= exit_s + self._c_exit_clear:
                return True, f"OK {leg} complete (exit marker at {exit_s:.1f} m)"
            if walls_end_s is not None and s >= walls_end_s + self._c_exit_clear:
                return True, f"OK {leg} complete (walls ended at {walls_end_s:.1f} m)"
            if s >= length + self._c_exit_clear and \
                    (not track.valid(t0) or not self._c_need_perception):
                if self._c_need_perception:
                    self.get_logger().warn(f"[path_planner] {leg}: no exit marker — exit by distance")
                return True, f"OK {leg} complete (by distance {s:.1f} m)"
            if s >= length + self._c_overrun:
                self.get_logger().warn(
                    f"[path_planner] {leg}: {s:.1f} m flown, exit not detected — leaving corridor")
                return True, f"OK {leg} complete (overrun {s:.1f} m)"

            # --- corridor not in view yet (at the entry point) ---
            healthy = track.valid(t0) and self._sensor_ok()
            if not started:
                # Both walls are not required: GPS error can put the vehicle
                # in line with one wall, which is then seen end-on. A
                # consistent fit over track_confirm frames is.
                if healthy and track.streak >= fp.track_confirm:
                    started = True
                    self.get_logger().info(
                        f"[path_planner] {leg}: corridor in view (width {track.width:.2f} m, "
                        f"offset {-track.centre:+.2f} m) — following")
                elif t0 > wait_until:
                    return False, f"NO_CORRIDOR {leg}: walls not seen within {self._c_entry_timeout:.0f}s"
                else:
                    self._sleep_rest(t0)
                    continue

            # --- last metres: walls end ahead, too short to fit -> fly out ---
            clear_to_go = last_cmd is not None and last_cmd.advance >= fp.lookahead_m - 1e-6
            if not healthy and self._sensor_ok() and walls_end_s is not None \
                    and clear_to_go and walls_end_s - s <= self._c_exit_run_m:
                if last_mode != "exit":
                    self.get_logger().info(
                        f"[path_planner] {leg}: walls end at {walls_end_s:.1f} m — flying out")
                    last_mode = "exit"
                out_alt = last_cmd.alt if memory.active is not None else nominal
                fe, fn = math.cos(c_axis), math.sin(c_axis)
                step = fp.lookahead_m
                lat, lon = mav.offset_gps(self._gps[0], self._gps[1], fn * step, fe * step)
                mav.goto(self, lat, lon, out_alt, publish_count=1, yaw_enu=c_axis, log=False)
                self._sleep_rest(t0)
                continue

            # --- perception lost during the corridor: hold / back off / fail ---
            if not healthy:
                if bad_since is None:
                    bad_since = t0
                    self.get_logger().warn(
                        f"[path_planner] {leg}: corridor perception lost — holding")
                    self._hold(alt, c_axis)
                elif not backed_off and t0 - bad_since >= self.cfg.coverage_backoff_after_s:
                    back = self.cfg.coverage_backoff_m
                    lat, lon = mav.offset_gps(self._gps[0], self._gps[1],
                                              -axis[0] * back, -axis[1] * back)
                    self.get_logger().warn(f"[path_planner] {leg}: backing off {back:.1f} m")
                    mav.goto(self, lat, lon, alt, publish_count=5, yaw_enu=c_axis)
                    backed_off = True
                if t0 - bad_since > self.cfg.sensor_fail_timeout_s:
                    return False, f"SENSOR_TIMEOUT {leg} at s={s:.1f} m"
                # The leg deadline is NOT extended here: lose/recover cycles
                # must not keep the vehicle in the corridor indefinitely.
                self._sleep_rest(t0)
                continue
            if bad_since is not None:
                self.get_logger().info(f"[path_planner] {leg}: perception back — resuming")
                bad_since, backed_off = None, False

            # --- plan and command ---
            cmd = last_cmd = plan(track, memory.confirmed(), s, alt, nominal, fp,
                                  mouth_s=mouth_s)
            if cmd.mode != last_mode:
                o = memory.active
                self.get_logger().info(
                    f"[path_planner] {leg}: {cmd.mode} at s={s:.1f} m"
                    + ("" if o is None else
                       f" (obstacle at {o.s_front:.1f} m, [{o.right:+.2f},{o.left:+.2f}] m"
                       f", top {o.top:.2f} m)"))
                last_mode = cmd.mode
            if cmd.mode == "blocked":
                blocked_since = blocked_since or t0
                if t0 - blocked_since > self.cfg.sensor_fail_timeout_s:
                    return False, f"BLOCKED {leg} at s={s:.1f} m: no side gap and too tall to climb"
            else:
                blocked_since = None

            heading = c_axis                            # corridor direction, ENU
            fe, fn = math.cos(heading), math.sin(heading)
            d_e = fe * cmd.advance - fn * cmd.lateral
            d_n = fn * cmd.advance + fe * cmd.lateral
            lat, lon = mav.offset_gps(self._gps[0], self._gps[1], d_n, d_e)
            mav.goto(self, lat, lon, cmd.alt, publish_count=1, yaw_enu=heading, log=False)
            self._sleep_rest(t0)

    def _sleep_rest(self, t0):
        time.sleep(max(0.0, self._c_period - (time.time() - t0)))

    # ===================================================================
    # Service handlers
    # ===================================================================

    def _respond(self, response, ok_msg):
        ok, msg = ok_msg
        response.success, response.message = ok, msg
        # Separate call sites: rclpy rejects one call site logging at two
        # different severities.
        if ok:
            self.get_logger().info(f"[path_planner] -> {msg}")
        else:
            self.get_logger().warn(f"[path_planner] -> {msg}")
        return response

    def _handle_corridor_1(self, request, response):
        return self._respond(response, self._run(
            "corridor_1", self.cfg.corridor_timeout_s,
            lambda: self._corridor("corridor_1", self._corridor1_local)))

    def _handle_corridor_2(self, request, response):
        return self._respond(response, self._run(
            "corridor_2", self.cfg.corridor_timeout_s,
            lambda: self._corridor("corridor_2", self._corridor2_local)))

    def _handle_run_lawnmower(self, request, response):
        return self._respond(response, self._run(
            "lawnmower", self.cfg.lawnmower_timeout_s, self._lawnmower))

    def _handle_stop(self, request, response):
        self._cancel.set()
        # Hold here before replying so the active leg's last target is
        # replaced; the caller's next goto then overrides this hold.
        self._hold(self._rel_alt if self._rel_alt is not None else self.cfg.cruise_altitude_m)
        response.success = True
        response.message = "OK stopped, holding position"
        self.get_logger().info("[path_planner] STOP — holding position")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[path_planner] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
