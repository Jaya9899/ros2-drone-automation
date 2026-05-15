# path_planner_node.py — ROS2 node for corridor navigation and lawnmower search
#
# Provides Trigger services consumed by the mission_manager:
#   /path_planner/fly_corridor     — fly through corridor waypoints via Nav2
#   /path_planner/run_lawnmower    — execute BCD lawnmower pattern via Nav2
#   /path_planner/stop_lawnmower   — cancel current lawnmower execution
#
# Navigation strategy:
#   Uses Nav2 FollowWaypoints action for lawnmower search.
#   Uses Nav2 NavigateThroughPoses action for corridor transit.
#   Falls back to direct MAVROS waypoint publishing if Nav2 is unavailable.
#
# Usage:
#   ros2 run path_planner path_planner_node
#   ros2 run path_planner path_planner_node --ros-args \
#       -p origin_lat:=12.9716 -p origin_lon:=77.5946

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

# Nav2 action types
from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses

from path_planner.waypoint_gen import (
    generate_mission_waypoints,
    local_to_gps,
)

from mission_manager.config_loader import load_mission_config


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")

        # ----- Load config -----
        self.cfg = load_mission_config(self)

        # ----- Node-specific parameters -----
        try:
            self.declare_parameter("use_nav2", True)
        except Exception:
            pass
        self.use_nav2 = self.get_parameter("use_nav2").value

        # Corridor waypoints from YAML (parallel x/y lists)
        try:
            self.declare_parameter(
                "corridor_waypoints_x", [0.0, 2.5, 5.0, 7.5, 10.0])
            self.declare_parameter(
                "corridor_waypoints_y", [15.0, 15.0, 15.0, 15.0, 15.0])
        except Exception:
            pass
        corr_x = self.get_parameter("corridor_waypoints_x").value
        corr_y = self.get_parameter("corridor_waypoints_y").value
        self._corridor_local = list(zip(corr_x, corr_y))

        # Red zone vertices from YAML (parallel x/y lists)
        try:
            self.declare_parameter("red_zone_x", [24.0, 36.0, 36.0, 24.0])
            self.declare_parameter("red_zone_y", [12.0, 12.0, 22.0, 22.0])
        except Exception:
            pass
        rz_x = self.get_parameter("red_zone_x").value
        rz_y = self.get_parameter("red_zone_y").value
        self._red_zone_vertices = list(zip(rz_x, rz_y))

        # ----- Callback group for concurrent service handling -----
        self._cb_group = ReentrantCallbackGroup()

        # ----- Nav2 action clients -----
        self._follow_wp_client = ActionClient(
            self, FollowWaypoints, "follow_waypoints",
            callback_group=self._cb_group,
        )
        self._nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses",
            callback_group=self._cb_group,
        )

        # ----- MAVROS fallback publisher -----
        try:
            from mavros_msgs.msg import GlobalPositionTarget
            self._gpt_type = GlobalPositionTarget
            self._mavros_pub = self.create_publisher(
                GlobalPositionTarget,
                "/mavros/setpoint_position/global",
                10,
            )
            self._has_mavros = True
        except ImportError:
            self._has_mavros = False
            self._mavros_pub = None

        # ----- Internal state -----
        self._lawnmower_active = False
        self._lawnmower_goal_handle = None
        self._corridor_goal_handle = None

        # ----- Pre-generate lawnmower waypoints -----
        self._mission_wps = generate_mission_waypoints(
            origin_lat=self.cfg.origin_lat,
            origin_lon=self.cfg.origin_lon,
            arena_width=self.cfg.arena_width_m,
            arena_height=self.cfg.arena_height_m,
            altitude=self.cfg.cruise_altitude_m,
            hfov_deg=self.cfg.oak_hfov_deg,
            overlap=self.cfg.strip_overlap,
            red_zone_vertices=self._red_zone_vertices,
            red_zone_buffer=self.cfg.red_zone_buffer_m,
            entry_x=self.cfg.corridor_entry_x,
            entry_y=self.cfg.corridor_entry_y,
        )
        self.get_logger().info(
            f"[path_planner] Generated {len(self._mission_wps['gps_waypoints'])} "
            f"lawnmower waypoints, strip={self._mission_wps['strip_width']:.1f}m, "
            f"est. dist={self._mission_wps['estimated_dist']:.0f}m, "
            f"est. time={self._mission_wps['estimated_time']/60:.1f}min"
        )

        # ----- Convert corridor waypoints to GPS -----
        self._corridor_gps = [
            local_to_gps(x, y, self.cfg.origin_lat, self.cfg.origin_lon)
            for x, y in self._corridor_local
        ]

        # ----- Services -----
        self.create_service(
            Trigger, "/path_planner/fly_corridor",
            self._handle_fly_corridor,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/path_planner/run_lawnmower",
            self._handle_run_lawnmower,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/path_planner/stop_lawnmower",
            self._handle_stop_lawnmower,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "[path_planner] Ready — services: fly_corridor, run_lawnmower, stop_lawnmower"
        )

    # ===================================================================
    # Helpers: convert GPS waypoints to Nav2 PoseStamped
    # ===================================================================

    def _gps_to_pose_stamped(self, lat: float, lon: float,
                             alt: float = None) -> PoseStamped:
        """
        Convert a GPS (lat, lon) to a PoseStamped in the map frame.

        For Nav2, we convert GPS to local ENU offsets from origin and
        create a PoseStamped at that position.
        """
        if alt is None:
            alt = self.cfg.cruise_altitude_m

        # Convert GPS to local ENU
        dlat = lat - self.cfg.origin_lat
        dlon = lon - self.cfg.origin_lon
        METRES_PER_DEG_LAT = 111_320.0
        metres_per_deg_lon = 111_320.0 * math.cos(
            math.radians(self.cfg.origin_lat))

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = dlon * metres_per_deg_lon   # East
        pose.pose.position.y = dlat * METRES_PER_DEG_LAT   # North
        pose.pose.position.z = alt
        pose.pose.orientation.w = 1.0  # facing forward (yaw=0)
        return pose

    # ===================================================================
    # Helpers: wait for futures
    # ===================================================================

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """Wait for a future to complete without spinning the node."""
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout_sec:
                return False
            time.sleep(0.05)
        return True

    # ===================================================================
    # Nav2 FollowWaypoints — for lawnmower
    # ===================================================================

    def _send_follow_waypoints(self, gps_waypoints: list,
                               timeout: float) -> bool:
        """Send a FollowWaypoints goal to Nav2 and block until completion."""
        if not self._follow_wp_client.wait_for_server(
            timeout_sec=self.cfg.nav2_connect_timeout_s
        ):
            self.get_logger().warn(
                "[path_planner] Nav2 FollowWaypoints server not available"
            )
            return False

        # Build goal
        goal = FollowWaypoints.Goal()
        goal.poses = [
            self._gps_to_pose_stamped(lat, lon) for lat, lon in gps_waypoints
        ]

        self.get_logger().info(
            f"[path_planner] Sending {len(goal.poses)} waypoints to Nav2 FollowWaypoints"
        )

        send_future = self._follow_wp_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 10.0):
            self.get_logger().error("[path_planner] FollowWaypoints send_goal timed out")
            return False

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("[path_planner] Nav2 rejected FollowWaypoints goal")
            return False

        self._lawnmower_goal_handle = goal_handle
        self.get_logger().info("[path_planner] Nav2 accepted FollowWaypoints goal")

        # Wait for result
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout):
            self.get_logger().error("[path_planner] FollowWaypoints result timed out")
            return False

        self._lawnmower_goal_handle = None

        if result_future.result() is None:
            self.get_logger().warn("[path_planner] FollowWaypoints timed out")
            return False

        result = result_future.result().result
        missed = result.missed_waypoints
        if missed:
            self.get_logger().warn(
                f"[path_planner] FollowWaypoints missed {len(missed)} waypoints: {missed}"
            )
        else:
            self.get_logger().info("[path_planner] FollowWaypoints completed successfully")

        return True

    # ===================================================================
    # Nav2 NavigateThroughPoses — for corridor transit
    # ===================================================================

    def _send_navigate_through_poses(self, gps_waypoints: list,
                                     timeout: float) -> bool:
        """Send a NavigateThroughPoses goal to Nav2 and block until completion."""
        if not self._nav_through_poses_client.wait_for_server(
            timeout_sec=self.cfg.nav2_connect_timeout_s
        ):
            self.get_logger().warn(
                "[path_planner] Nav2 NavigateThroughPoses server not available"
            )
            return False

        goal = NavigateThroughPoses.Goal()
        goal.poses = [
            self._gps_to_pose_stamped(lat, lon) for lat, lon in gps_waypoints
        ]

        self.get_logger().info(
            f"[path_planner] Sending {len(goal.poses)} poses to Nav2 NavigateThroughPoses"
        )

        send_future = self._nav_through_poses_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 10.0):
            self.get_logger().error("[path_planner] NavigateThroughPoses send_goal timed out")
            return False

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                "[path_planner] Nav2 rejected NavigateThroughPoses goal"
            )
            return False

        self._corridor_goal_handle = goal_handle
        self.get_logger().info("[path_planner] Nav2 accepted corridor navigation goal")

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout):
            self.get_logger().error("[path_planner] NavigateThroughPoses result timed out")
            return False

        self._corridor_goal_handle = None

        if result_future.result() is None:
            self.get_logger().warn("[path_planner] NavigateThroughPoses timed out")
            return False

        self.get_logger().info("[path_planner] Corridor navigation completed")
        return True

    # ===================================================================
    # MAVROS fallback — direct waypoint publishing
    # ===================================================================

    def _fly_waypoints_mavros(self, gps_waypoints: list,
                              speed_ms: float, timeout: float) -> bool:
        """
        Fly through waypoints using direct MAVROS global position setpoints.
        Used as fallback when Nav2 is unavailable.
        """
        if not self._has_mavros:
            self.get_logger().warn(
                "[path_planner] No MAVROS publisher — simulating waypoint flight"
            )
            for i, (lat, lon) in enumerate(gps_waypoints):
                self.get_logger().info(
                    f"[path_planner] [SIM] Flying to WP{i}: ({lat:.7f}, {lon:.7f})"
                )
                time.sleep(0.5)  # simulate transit time
            return True

        from mavros_msgs.msg import GlobalPositionTarget
        from std_msgs.msg import Header
        from sensor_msgs.msg import NavSatFix

        arrival_tolerance = self.cfg.mavros_arrival_tolerance_m

        for i, (lat, lon) in enumerate(gps_waypoints):
            self.get_logger().info(
                f"[path_planner] MAVROS WP{i}/{len(gps_waypoints)-1}: "
                f"({lat:.7f}, {lon:.7f})"
            )

            # Publish setpoint
            msg = GlobalPositionTarget()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            msg.type_mask = (
                GlobalPositionTarget.IGNORE_VX |
                GlobalPositionTarget.IGNORE_VY |
                GlobalPositionTarget.IGNORE_VZ |
                GlobalPositionTarget.IGNORE_AFX |
                GlobalPositionTarget.IGNORE_AFY |
                GlobalPositionTarget.IGNORE_AFZ |
                GlobalPositionTarget.IGNORE_YAW_RATE
            )
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = self.cfg.cruise_altitude_m

            # Keep publishing until we arrive at this waypoint
            arrived = False
            deadline = time.time() + timeout / len(gps_waypoints)

            def _haversine(lat1, lon1, lat2, lon2):
                R = 6_371_000.0
                phi1, phi2 = math.radians(lat1), math.radians(lat2)
                dphi = math.radians(lat2 - lat1)
                dlam = math.radians(lon2 - lon1)
                a = (math.sin(dphi / 2) ** 2 +
                     math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
                return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

            current_pos = [None, None]  # [lat, lon]

            def _gps_cb(nav_msg: NavSatFix):
                current_pos[0] = nav_msg.latitude
                current_pos[1] = nav_msg.longitude

            sub = self.create_subscription(
                NavSatFix, "/mavros/global_position/global", _gps_cb, 10
            )

            while not arrived and time.time() < deadline:
                msg.header.stamp = self.get_clock().now().to_msg()
                self._mavros_pub.publish(msg)
                time.sleep(0.25)

                if current_pos[0] is not None:
                    dist = _haversine(current_pos[0], current_pos[1], lat, lon)
                    if dist < arrival_tolerance:
                        arrived = True

                if not self._lawnmower_active and i > 0:
                    # Lawnmower was cancelled
                    self.destroy_subscription(sub)
                    return False

            self.destroy_subscription(sub)

            if not arrived:
                self.get_logger().warn(
                    f"[path_planner] Timeout reaching WP{i}"
                )

        return True

    # ===================================================================
    # Service handlers
    # ===================================================================

    def _handle_fly_corridor(self, request, response):
        """Fly through corridor waypoints."""
        self.get_logger().info("[path_planner] FLY_CORRIDOR requested")

        corridor_wps = list(self._corridor_gps)

        if self.use_nav2:
            ok = self._send_navigate_through_poses(
                corridor_wps, self.cfg.corridor_timeout_s)
        else:
            ok = self._fly_waypoints_mavros(
                corridor_wps, self.cfg.corridor_speed_ms,
                self.cfg.corridor_timeout_s)

        if not ok:
            # Try MAVROS fallback if Nav2 failed
            if self.use_nav2 and self._has_mavros:
                self.get_logger().warn(
                    "[path_planner] Nav2 failed, falling back to MAVROS"
                )
                ok = self._fly_waypoints_mavros(
                    corridor_wps, self.cfg.corridor_speed_ms,
                    self.cfg.corridor_timeout_s)

        response.success = ok
        response.message = "Corridor transit complete" if ok else "Corridor transit failed"
        return response

    def _handle_run_lawnmower(self, request, response):
        """Execute BCD lawnmower search pattern."""
        self.get_logger().info("[path_planner] RUN_LAWNMOWER requested")

        self._lawnmower_active = True
        gps_wps = self._mission_wps['gps_waypoints']

        if self.use_nav2:
            ok = self._send_follow_waypoints(
                gps_wps, self.cfg.lawnmower_timeout_s)
        else:
            ok = self._fly_waypoints_mavros(
                gps_wps, self.cfg.arena_speed_ms,
                self.cfg.lawnmower_timeout_s)

        if not ok and self.use_nav2 and self._has_mavros:
            self.get_logger().warn(
                "[path_planner] Nav2 failed, falling back to MAVROS"
            )
            ok = self._fly_waypoints_mavros(
                gps_wps, self.cfg.arena_speed_ms,
                self.cfg.lawnmower_timeout_s)

        self._lawnmower_active = False

        response.success = ok
        response.message = (
            f"Lawnmower complete ({len(gps_wps)} waypoints)"
            if ok else "Lawnmower failed or cancelled"
        )
        return response

    def _handle_stop_lawnmower(self, request, response):
        """Cancel current lawnmower execution."""
        self.get_logger().info("[path_planner] STOP_LAWNMOWER requested")

        self._lawnmower_active = False

        # Cancel Nav2 goal if active
        if self._lawnmower_goal_handle is not None:
            self.get_logger().info("[path_planner] Cancelling Nav2 FollowWaypoints goal")
            cancel_future = self._lawnmower_goal_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, 5.0)
            self._lawnmower_goal_handle = None

        response.success = True
        response.message = "Lawnmower stopped"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        # Use MultiThreadedExecutor for concurrent service handling
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[path_planner] Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
