# path_planner_node.py — ROS2 node for corridor navigation and lawnmower search
#
# Provides Trigger services consumed by the mission_manager:
#   /path_planner/fly_corridor     — fly through corridor waypoints via MAVROS
#   /path_planner/run_lawnmower    — execute BCD lawnmower pattern via MAVROS
#   /path_planner/stop_lawnmower   — cancel current lawnmower execution
#
# Navigation strategy:
#   Uses direct MAVROS waypoint publishing for all navigation.
#   RL-based obstacle avoidance can be enabled via use_rl_avoidance param
#   (placeholder — RL model integration pending).
#
# Usage:
#   ros2 run path_planner path_planner_node
#   ros2 run path_planner path_planner_node --ros-args \
#       -p origin_lat:=12.9716 -p origin_lon:=77.5946

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

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
            self.declare_parameter("red_zone_1_x", [24.0, 36.0, 36.0, 24.0])
            self.declare_parameter("red_zone_1_y", [12.0, 12.0, 22.0, 22.0])
            self.declare_parameter("red_zone_2_x", [10.0, 15.0, 12.5])
            self.declare_parameter("red_zone_2_y", [5.0, 5.0, 10.0])
            self.declare_parameter("red_zone_3_x", [5.0, 10.0, 10.0, 7.0, 7.0, 5.0])
            self.declare_parameter("red_zone_3_y", [20.0, 20.0, 22.0, 22.0, 25.0, 25.0])
        except Exception:
            pass
        self._red_zones = []
        for i in range(1, 4):
            try:
                rz_x = self.get_parameter(f"red_zone_{i}_x").value
                rz_y = self.get_parameter(f"red_zone_{i}_y").value
                if rz_x and rz_y and len(rz_x) == len(rz_y):
                    self._red_zones.append(list(zip(rz_x, rz_y)))
            except Exception:
                pass

        # ----- Callback group for concurrent service handling -----
        self._cb_group = ReentrantCallbackGroup()

        # ----- MAVROS publisher -----
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

        # ----- RL obstacle avoidance (placeholder) -----
        self._setup_rl_avoidance()

        # ----- Internal state -----
        self._lawnmower_active = False

        # ----- Generate mission waypoints -----
        self.current_gps_pos = [None, None]  # [lat, lon]

        self._mission_wps = generate_mission_waypoints(
            origin_lat=self.cfg.origin_lat,
            origin_lon=self.cfg.origin_lon,
            arena_width=self.cfg.arena_width_m,
            arena_height=self.cfg.arena_height_m,
            altitude=self.cfg.cruise_altitude_m,
            hfov_deg=self.cfg.oak_hfov_deg,
            overlap=self.cfg.strip_overlap,
            red_zones_vertices=self._red_zones,
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

        # ----- GPS subscriber (for position tracking) -----
        self.current_gps_pos = [None, None]
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._gps_cb, mavros_qos
        )

    # ===================================================================
    # RL Obstacle Avoidance — placeholder setup
    # ===================================================================

    def _setup_rl_avoidance(self):
        """
        Set up RL-based obstacle avoidance interfaces.

        When use_rl_avoidance is True, the path planner will:
          1. Publish sensor data to the RL model via /rl_obstacle_avoidance/sensor_input
          2. Subscribe to velocity overrides from the RL model
          3. Apply velocity corrections before sending waypoints to MAVROS

        TODO: Integrate trained RL model. Current implementation is a
              pass-through placeholder.
        """
        self._rl_avoidance_enabled = self.cfg.use_rl_avoidance
        self._rl_velocity_override = None  # Latest override from RL model

        if self._rl_avoidance_enabled:
            self.get_logger().info(
                "[path_planner] RL obstacle avoidance ENABLED "
                f"(sensor: {self.cfg.rl_sensor_topic})"
            )

            # Subscribe to RL model velocity overrides
            self._rl_vel_sub = self.create_subscription(
                TwistStamped,
                "/rl_obstacle_avoidance/velocity_override",
                self._rl_velocity_cb,
                10,
                callback_group=self._cb_group,
            )

            # Publisher to forward sensor data to RL model
            # (The RL node should subscribe directly to the sensor topic,
            #  but this publisher can be used for pre-processed data)
            self._rl_status_pub = self.create_publisher(
                PoseStamped,
                "/rl_obstacle_avoidance/current_goal",
                10,
            )
        else:
            self.get_logger().info(
                "[path_planner] RL obstacle avoidance DISABLED "
                "(using direct MAVROS waypoint navigation)"
            )

    def _rl_velocity_cb(self, msg: TwistStamped):
        """Receive velocity override from RL obstacle avoidance model."""
        self._rl_velocity_override = msg

    def _apply_rl_avoidance(self, lat: float, lon: float, alt: float):
        """
        Apply RL-based obstacle avoidance corrections before navigating
        to a waypoint.

        TODO: Implement actual RL model inference here. The model should:
          1. Read current depth/pointcloud data from the sensor topic
          2. Predict whether the current path is obstructed
          3. Output a corrected velocity vector or adjusted waypoint
          4. Return the (possibly modified) target waypoint

        Current implementation: pass-through (no modification).

        Args:
            lat: Target waypoint latitude
            lon: Target waypoint longitude
            alt: Target waypoint altitude

        Returns:
            Tuple of (lat, lon, alt) — possibly modified by RL model
        """
        if not self._rl_avoidance_enabled:
            return lat, lon, alt

        # Publish current goal for RL model awareness
        if hasattr(self, '_rl_status_pub'):
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = lon  # placeholder mapping
            goal_msg.pose.position.y = lat
            goal_msg.pose.position.z = alt
            self._rl_status_pub.publish(goal_msg)

        # TODO: Query RL model for obstacle avoidance correction
        # If self._rl_velocity_override is set, apply it to modify
        # the target waypoint or velocity command.
        #
        # Example future implementation:
        #   if self._rl_velocity_override is not None:
        #       override = self._rl_velocity_override
        #       # Apply velocity correction to adjust waypoint
        #       lat += override.twist.linear.y * dt
        #       lon += override.twist.linear.x * dt
        #       self._rl_velocity_override = None

        return lat, lon, alt

    # ===================================================================
    # GPS callback
    # ===================================================================

    def _gps_cb(self, nav_msg: NavSatFix):
        self.current_gps_pos[0] = nav_msg.latitude
        self.current_gps_pos[1] = nav_msg.longitude

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
    # MAVROS waypoint navigation
    # ===================================================================

    def _fly_waypoints_mavros(self, gps_waypoints: list,
                              speed_ms: float, timeout: float) -> bool:
        """
        Fly through waypoints using mavros_utils.goto and wait_for_arrival.
        If RL avoidance is enabled, each waypoint is passed through the
        RL correction layer before being sent to MAVROS.
        """
        import sys
        # Ensure mission_manager is in path for imports
        sys.path.insert(0, '/home/jaya9899/ros2_ws/install/mission_manager/lib/python3.10/site-packages')
        from mission_manager.mavros_utils import goto, wait_for_arrival

        # At least 16s per waypoint to give SITL time to manoeuvre
        per_wp_timeout = max(timeout / max(len(gps_waypoints), 1), 16.0)

        for i, (lat, lon) in enumerate(gps_waypoints):
            if not self._lawnmower_active and i > 0:
                return False

            # Apply RL obstacle avoidance correction (placeholder)
            lat, lon, alt = self._apply_rl_avoidance(
                lat, lon, self.cfg.cruise_altitude_m)

            self.get_logger().info(
                f"[path_planner] WP{i+1}/{len(gps_waypoints)}: "
                f"({lat:.7f}, {lon:.7f})"
            )

            # Send waypoint command (higher publish count for reliability)
            goto(self, lat=lat, lon=lon,
                 altitude_m=alt,
                 publish_count=20)

            # Wait for arrival
            arrived = wait_for_arrival(
                self, lat=lat, lon=lon,
                tolerance_m=self.cfg.mavros_arrival_tolerance_m,
                timeout=per_wp_timeout,
            )

            if not arrived:
                self.get_logger().warn(
                    f"[path_planner] Timeout reaching WP{i+1}, continuing to next"
                )

        return True

    # ===================================================================
    # Service handlers
    # ===================================================================

    def _handle_fly_corridor(self, request, response):
        """Fly through corridor waypoints. Blocks until navigation completes."""
        self.get_logger().info("[path_planner] FLY_CORRIDOR requested")
        corridor_wps = list(self._corridor_gps)

        if not rclpy.ok():
            response.success = False
            response.message = "Node shutting down"
            return response

        ok = self._fly_waypoints_mavros(
            corridor_wps, self.cfg.corridor_speed_ms,
            self.cfg.corridor_timeout_s)

        self.get_logger().info(f"[path_planner] FLY_CORRIDOR finished (ok={ok})")
        response.success = ok
        response.message = f"Corridor transit {'completed' if ok else 'failed'}"
        return response

    def _handle_run_lawnmower(self, request, response):
        """Execute BCD lawnmower search pattern. Blocks until complete or stopped."""
        self.get_logger().info("[path_planner] RUN_LAWNMOWER requested")
        self._lawnmower_active = True
        gps_wps = self._mission_wps['gps_waypoints']

        if not rclpy.ok():
            self._lawnmower_active = False
            response.success = False
            response.message = "Node shutting down"
            return response

        ok = self._fly_waypoints_mavros(
            gps_wps, self.cfg.arena_speed_ms,
            self.cfg.lawnmower_timeout_s)

        self._lawnmower_active = False
        self.get_logger().info(f"[path_planner] RUN_LAWNMOWER finished (ok={ok})")
        response.success = ok
        response.message = f"Lawnmower {'completed' if ok else 'stopped/failed'}"
        return response

    def _handle_stop_lawnmower(self, request, response):
        """Cancel current lawnmower execution."""
        self.get_logger().info("[path_planner] STOP_LAWNMOWER requested")
        self._lawnmower_active = False
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
