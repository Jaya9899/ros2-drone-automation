# path_planner_node.py — ROS2 node that flies the mission legs via MAVROS
#
# Trigger services consumed by mission_manager (each blocks until the leg is
# done, cancelled, or failed):
#   /path_planner/fly_corridor_1   — clean corridor   (corridor_speed_ms)
#   /path_planner/fly_corridor_2   — obstacle corridor (corridor_obs_speed_ms)
#   /path_planner/run_lawnmower    — BCD lawnmower search (arena_speed_ms)
#   /path_planner/stop             — cancel the active leg and hold position
#   /path_planner/stop_lawnmower   — alias of stop (backwards compatible)
#
# Response message is a machine-readable token first:
#   OK | CANCELLED | BUSY | NO_GPS | WP_TIMEOUT ... | SENSOR_TIMEOUT ...
#
# Navigation: GUIDED GlobalPositionTarget setpoints (mavros_utils.goto), nose
# pointed along the direction of travel so the forward OAK-D sees ahead.
#
# ArduPilot 4.7.1 GUIDED has two ways to fly a position target, chosen by
# GUID_OPTIONS bit 6 (mode_guided.cpp set_destination):
#   bit 6 clear -> position controller straight to the target. Speed follows
#                  MAV_CMD_DO_CHANGE_SPEED. NO object avoidance (BendyRuler
#                  lives only in the waypoint navigator, AC_WPNav_OA).
#   bit 6 set   -> waypoint navigator (S-curves) WITH BendyRuler. Speed is the
#                  WP_SPD param read when this mode starts; DO_CHANGE_SPEED is
#                  overwritten on every new target.
# So corridor 2 (obstacles) runs with bit 6 set and WP_SPD = its speed; every
# other leg runs with bit 6 clear and DO_CHANGE_SPEED. Each leg first sends a
# hold target: entering a submode resets the speed limits to WP_SPD, so the
# speed must be set AFTER that first target.
#
# Obstacle-sensor gating (coverage_guard topics), per leg:
#   sensor_live False           -> hold position; resume when live again
#   coverage_ok False (corr. 2) -> hold; after coverage_backoff_after_s back off
#                                  coverage_backoff_m along the corridor; resume
#                                  when coverage recovers
#   unhealthy > sensor_fail_timeout_s -> fail the leg with SENSOR_TIMEOUT
#                                        (mission_node then LANDs)
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
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix, NavSatStatus

from mission_manager import mavros_utils as mav
from mission_manager.config_loader import load_mission_config
from path_planner.waypoint_gen import (
    build_transit_space,
    generate_mission_waypoints,
    gps_to_local,
    local_to_gps,
    plan_path,
)

_DOUBLE_ARRAY = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
GUID_OPT_WPNAV_FOR_POSITION = 1 << 6      # "Waypoint navigation used for position targets"


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")
        self.cfg = load_mission_config(self)

        # ----- Course geometry (local ENU metres about origin) -----
        # PLACEHOLDERS — set the real corridors on competition day (yaml).
        self._corridor1_local = self._xy_param(
            "corridor1_waypoints", [0.0, 2.5, 5.0, 7.5, 10.0], [15.0] * 5)
        self._corridor2_local = self._xy_param(
            "corridor2_waypoints", [10.0, 7.5, 5.0, 2.5, 0.0], [-5.0] * 5)

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
        # BendyRuler for corridor 2 (see header). guided_options_base holds any
        # other GUID_OPTIONS bits you want set on the FC; bit 6 is managed here.
        self.declare_parameter("corridor2_bendyruler", True)
        self.declare_parameter("guided_options_base", 0)
        self._corridor2_oa = bool(self.get_parameter("corridor2_bendyruler").value)
        self._guid_options_base = int(self.get_parameter("guided_options_base").value) \
            & ~GUID_OPT_WPNAV_FOR_POSITION
        self._guid_options = None               # last value written to the FC
        self._leg_oa = False                    # BendyRuler active on this leg
        edge_margin = float(self.get_parameter("arena_edge_margin_m").value)

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
            entry_x=self.cfg.corridor_entry_x,
            entry_y=self.cfg.corridor_entry_y,
            speed_ms=self.cfg.arena_speed_ms,
            edge_margin=edge_margin,
        )
        self._lawnmower_local = wps["local_waypoints"]
        self.get_logger().info(
            f"[path_planner] {len(self._lawnmower_local)} lawnmower waypoints, "
            f"strip={wps['strip_width']:.1f} m, est. {wps['estimated_dist']:.0f} m / "
            f"{wps['estimated_time'] / 60:.1f} min")

        # Free space for transit hops (corridor exits, drop point -> corridor 2).
        self._transit_space = build_transit_space(
            self._corridor1_local + self._corridor2_local + self._lawnmower_local,
            self._red_zones, self.cfg.red_zone_buffer_m)

        # ----- Live data -----
        self._gps = None
        self._sensor_live = False
        self._coverage_ok = False

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
        self.create_subscription(Bool, "/avoidance/sensor_live", self._live_cb, 10)
        self.create_subscription(Bool, "/avoidance/coverage_ok", self._cov_cb, 10)

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
        self.create_service(Trigger, "/path_planner/stop_lawnmower",
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

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self._gps = (msg.latitude, msg.longitude)

    def _live_cb(self, msg: Bool):
        self._sensor_live = bool(msg.data)

    def _cov_cb(self, msg: Bool):
        self._coverage_ok = bool(msg.data)

    # ===================================================================
    # Leg execution
    # ===================================================================

    def _to_gps(self, xy):
        return local_to_gps(xy[0], xy[1], self.cfg.origin_lat, self.cfg.origin_lon)

    def _to_local(self, latlon):
        return gps_to_local(latlon[0], latlon[1], self.cfg.origin_lat, self.cfg.origin_lon)

    def _health(self, leg: str):
        """None if OK, else 'live' / 'coverage'."""
        if not self.cfg.require_avoidance_sensor:
            return None
        if not self._sensor_live:
            return "live"
        if leg == "corridor_2" and not self._coverage_ok:
            return "coverage"
        return None

    def _hold(self, alt):
        if self._gps is not None:
            mav.goto(self, self._gps[0], self._gps[1], alt, publish_count=5)

    def _goto(self, latlon, alt):
        yaw = None
        # With BendyRuler active, never pin the heading: the path bends away
        # from the straight line to the waypoint, and the forward-only OAK-D
        # (+-35 deg) must look where the vehicle is actually going. With yaw
        # ignored, GUIDED's waypoint navigator faces the (OA-adjusted) travel
        # direction itself. Pinning it toward the waypoint let the drone slide
        # sideways into a wall it could not see (SITL, 2026-10-03).
        if not self._leg_oa and self._gps is not None:
            yaw = mav.yaw_enu_towards(self._gps[0], self._gps[1], latlon[0], latlon[1])
        mav.goto(self, latlon[0], latlon[1], alt,
                 publish_count=self.cfg.goto_publish_count, yaw_enu=yaw)

    def _fly_to(self, latlon, prev_latlon, alt, speed, leg):
        """Fly one waypoint. Returns 'OK', 'CANCELLED', 'WP_TIMEOUT', 'SENSOR_TIMEOUT', 'LEG_TIMEOUT'."""
        dist = mav.haversine(*self._gps, *latlon)
        wp_deadline = time.time() + 10.0 + 2.5 * dist / max(speed, 0.1)
        self._goto(latlon, alt)

        bad_since = None
        backed_off = False
        while True:
            if self._cancel.is_set() or not rclpy.ok():
                return "CANCELLED"
            now = time.time()
            problem = self._health(leg)
            if problem is not None:
                if bad_since is None:
                    bad_since = now
                    self.get_logger().warn(
                        f"[path_planner] {leg}: obstacle sensor {problem} not OK — holding")
                    self._hold(alt)
                elif (problem == "coverage" and not backed_off
                      and now - bad_since >= self.cfg.coverage_backoff_after_s):
                    self._back_off(latlon, prev_latlon, alt)
                    backed_off = True
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
                backed_off = False
                self._goto(latlon, alt)

            if self._gps is not None and \
                    mav.haversine(*self._gps, *latlon) <= self.cfg.mavros_arrival_tolerance_m:
                return "OK"
            if now > self._leg_deadline:
                return "LEG_TIMEOUT"
            if now > wp_deadline:
                return "WP_TIMEOUT"
            time.sleep(0.2)

    def _back_off(self, target, prev, alt):
        """Retreat coverage_backoff_m back along the leg (from target toward prev)."""
        if self._gps is None:
            return
        ref = prev if prev is not None else self._gps
        d_n, d_e = mav.ne_offset(target[0], target[1], ref[0], ref[1])
        norm = math.hypot(d_n, d_e)
        if norm < 0.1:
            return
        step = self.cfg.coverage_backoff_m
        lat, lon = mav.offset_gps(self._gps[0], self._gps[1],
                                  d_n / norm * step, d_e / norm * step)
        self.get_logger().warn(f"[path_planner] Low depth coverage — backing off {step:.1f} m")
        mav.goto(self, lat, lon, alt, publish_count=10)   # keep current heading

    def _set_guided_options(self, use_oa: bool) -> bool:
        want = self._guid_options_base | (GUID_OPT_WPNAV_FOR_POSITION if use_oa else 0)
        if want == self._guid_options:
            return True
        if not mav.set_fcu_param(self, "GUID_OPTIONS", want):
            return False
        self._guid_options = want
        return True

    def _configure_leg(self, speed: float, use_oa: bool, alt: float) -> bool:
        if use_oa and not mav.set_fcu_param(self, "WP_SPD", float(speed)):
            return False
        if not self._set_guided_options(use_oa):
            return False
        self._hold(alt)                 # enter the submode (resets speed limits)...
        mav.change_speed(self, speed)   # ...then set the speed (position mode)
        return True

    def _run_leg(self, leg: str, local_wps: list, speed: float, timeout: float,
                 skip_timed_out_wps: bool, use_oa: bool = False):
        if not self._run_lock.acquire(blocking=False):
            return False, "BUSY another leg is active"
        try:
            self._cancel.clear()
            if self._gps is None:
                return False, "NO_GPS"
            alt = self.cfg.cruise_altitude_m
            self._leg_oa = use_oa
            if not self._configure_leg(speed, use_oa, alt):
                return False, f"FCU_CONFIG could not configure {leg} (GUID_OPTIONS/WP_SPD)"

            # Route from where we are to the leg start around red zones.
            start = self._to_local(self._gps)
            approach = plan_path(start, local_wps[0], self._transit_space)[1:-1]
            route = [self._to_gps(p) for p in approach + list(local_wps)]

            self._leg_deadline = time.time() + timeout
            prev = self._gps
            for i, wp in enumerate(route):
                self.get_logger().info(
                    f"[path_planner] {leg} WP{i + 1}/{len(route)}: ({wp[0]:.7f}, {wp[1]:.7f})")
                result = self._fly_to(wp, prev, alt, speed, leg)
                if result == "OK":
                    prev = wp
                    continue
                if result == "WP_TIMEOUT" and skip_timed_out_wps:
                    self.get_logger().warn(f"[path_planner] {leg}: WP{i + 1} timeout, skipping")
                    prev = wp
                    continue
                return False, f"{result} {leg} at WP{i + 1}/{len(route)}"
            return True, f"OK {leg} complete"
        finally:
            self._leg_oa = False
            if use_oa:
                # Back to plain position control for whatever comes next
                # (mission_node's drop/homing gotos assume it).
                self._set_guided_options(False)
            self._run_lock.release()

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
        # A corridor waypoint that cannot be reached means the vehicle is
        # stuck between walls — fail the leg rather than cut a corner.
        return self._respond(response, self._run_leg(
            "corridor_1", self._corridor1_local, self.cfg.corridor_speed_ms,
            self.cfg.corridor_timeout_s, skip_timed_out_wps=False))

    def _handle_corridor_2(self, request, response):
        return self._respond(response, self._run_leg(
            "corridor_2", self._corridor2_local, self.cfg.corridor_obs_speed_ms,
            self.cfg.corridor_timeout_s, skip_timed_out_wps=False,
            use_oa=self._corridor2_oa))

    def _handle_run_lawnmower(self, request, response):
        return self._respond(response, self._run_leg(
            "lawnmower", self._lawnmower_local, self.cfg.arena_speed_ms,
            self.cfg.lawnmower_timeout_s, skip_timed_out_wps=True))

    def _handle_stop(self, request, response):
        self._cancel.set()
        # Hold here before replying so the active leg's last target is
        # replaced; the caller's next goto then overrides this hold.
        self._hold(self.cfg.cruise_altitude_m)
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
