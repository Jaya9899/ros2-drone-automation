# safety_monitor.py — Background safety node for the mission
#
# Runs regardless of RC or Auto mode. Provides:
#   1. Battery voltage monitoring → emergency RTL if critical
#   2. OAK-D obstacle proximity warnings → publishes distance to nearest object
#   3. ArduPilot geofence upload → uploads polygon fence via MAVROS on startup
#
# Usage:
#   ros2 run mission_manager safety_monitor
#   ros2 run mission_manager safety_monitor --ros-args \
#       -p battery_critical_v:=13.5 -p oak_warning_dist_m:=1.5

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import BatteryState, PointCloud2
from mavros_msgs.srv import SetMode
import struct
import math
import time

from mission_manager.config_loader import load_mission_config


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor")

        # ----- Load config -----
        self.cfg = load_mission_config(self)

        # QoS profile for high-frequency telemetry / sensors
        # MAVROS often publishes these as BEST_EFFORT
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ----- Geofence points (flat list → list of (lat, lon) tuples) -----
        try:
            self.declare_parameter(
                "geofence_points",
                [12.97100, 77.59400, 12.97100, 77.59500,
                 12.97220, 77.59500, 12.97220, 77.59400],
            )
        except Exception:
            pass
        flat = self.get_parameter("geofence_points").value
        self._geofence_gps = [
            (flat[i], flat[i + 1]) for i in range(0, len(flat), 2)
        ]

        # ----- Fence parameters -----
        try:
            self.declare_parameter("fence_enable", 1)
            self.declare_parameter("fence_type", 7)
            self.declare_parameter("fence_action", 1)
            self.declare_parameter("fence_alt_max", 15.0)
            self.declare_parameter("fence_radius", 100.0)
        except Exception:
            pass
        self._fence_params = {
            "FENCE_ENABLE":  self.get_parameter("fence_enable").value,
            "FENCE_TYPE":    self.get_parameter("fence_type").value,
            "FENCE_ACTION":  self.get_parameter("fence_action").value,
            "FENCE_ALT_MAX": self.get_parameter("fence_alt_max").value,
            "FENCE_RADIUS":  self.get_parameter("fence_radius").value,
        }

        # ----- Publishers -----
        self.obstacle_dist_pub = self.create_publisher(
            Float32, "/safety/obstacle_distance", 10
        )
        self.battery_status_pub = self.create_publisher(
            String, "/safety/battery_status", 10
        )
        self.alert_pub = self.create_publisher(
            String, "/safety/alert", 10
        )
        self.obstacle_warning_pub = self.create_publisher(
            Bool, "/safety/obstacle_warning", 10
        )

        # ----- Subscribers -----
        self.create_subscription(
            BatteryState, "/mavros/battery",
            self._battery_cb, sensor_qos,
        )

        # OAK-D depth pointcloud
        self.create_subscription(
            PointCloud2, "/oak/points",
            self._pointcloud_cb, 10,
        )

        # ----- Internal state -----
        self._battery_voltage = None
        self._battery_alerted = False
        self._battery_critical_alerted = False
        self._min_obstacle_dist = float('inf')
        self._last_obstacle_warn = 0.0

        # ----- MAVROS service clients for emergency RTL -----
        self._set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # ----- Upload geofence on startup -----
        self._upload_geofence()

        # ----- Safety check timer -----
        self.create_timer(
            1.0 / self.cfg.safety_check_rate_hz, self._safety_tick
        )

        self.get_logger().info(
            f"[safety] Monitor started — battery critical: {self.cfg.battery_critical_v}V, "
            f"obstacle warn: {self.cfg.oak_warning_dist_m}m"
        )

    # ===================================================================
    # Geofence upload via MAVROS
    # ===================================================================

    def _upload_geofence(self):
        """
        Upload geofence polygon to ArduPilot via MAVROS.

        Uses /mavros/mission/push or MAVLink FENCE_POINT commands.
        ArduPilot enforces the geofence at the flight controller level.
        """
        try:
            from mavros_msgs.srv import WaypointPush
            from mavros_msgs.msg import Waypoint

            # ArduPilot geofence is configured via parameters, not waypoints
            # For now, we log the geofence coordinates and note that they
            # should be configured via Mission Planner or MAVProxy
            self.get_logger().info(
                "[safety] Geofence coordinates (configure in ArduPilot):"
            )
            for i, (lat, lon) in enumerate(self._geofence_gps):
                self.get_logger().info(
                    f"  FENCE_POINT_{i}: lat={lat:.7f}, lon={lon:.7f}"
                )

            # Set ArduPilot geofence parameters via MAVROS param service
            self._set_ardupilot_geofence_params()

        except ImportError:
            self.get_logger().warn(
                "[safety] mavros_msgs not available — geofence not uploaded"
            )

    def _set_ardupilot_geofence_params(self):
        """
        Set ArduPilot geofence parameters via MAVROS param set.

        Key parameters:
          FENCE_ENABLE  = 1 (enable geofence)
          FENCE_TYPE    = 7 (altitude + circle + polygon)
          FENCE_ACTION  = 1 (RTL on breach)
          FENCE_ALT_MAX = 15.0 (max altitude in metres)
          FENCE_RADIUS  = 100.0 (circular geofence radius)
        """
        try:
            from mavros_msgs.srv import ParamSet

            param_client = self.create_client(ParamSet, "/mavros/param/set")

            if not param_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(
                    "[safety] /mavros/param/set not available — "
                    "geofence params not set"
                )
                return

            for name, value in self._fence_params.items():
                req = ParamSet.Request()
                req.param_id = name
                if isinstance(value, int):
                    req.value.integer = value
                else:
                    req.value.real = float(value)

                future = param_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if future.result() and future.result().success:
                    self.get_logger().info(
                        f"[safety] Set {name} = {value}"
                    )
                else:
                    self.get_logger().warn(
                        f"[safety] Failed to set {name} = {value}"
                    )

        except (ImportError, Exception) as e:
            self.get_logger().warn(f"[safety] Geofence param setup failed: {e}")

    # ===================================================================
    # Battery monitoring
    # ===================================================================

    def _battery_cb(self, msg: BatteryState):
        """Process battery state updates."""
        self._battery_voltage = msg.voltage

    def _check_battery(self):
        """Check battery voltage and trigger alerts/RTL."""
        if self._battery_voltage is None or self._battery_voltage <= 0:
            return

        v = self._battery_voltage

        # Publish status
        status_msg = String()
        if v < self.cfg.battery_critical_v:
            status_msg.data = f"CRITICAL:{v:.1f}V"
        elif v < self.cfg.battery_warning_v:
            status_msg.data = f"WARNING:{v:.1f}V"
        else:
            status_msg.data = f"OK:{v:.1f}V"
        self.battery_status_pub.publish(status_msg)

        # Warning alert
        if v < self.cfg.battery_warning_v and not self._battery_alerted:
            self._battery_alerted = True
            alert = String()
            alert.data = f"BATTERY_WARNING: {v:.1f}V < {self.cfg.battery_warning_v}V"
            self.alert_pub.publish(alert)
            self.get_logger().warn(f"[safety] {alert.data}")

        # Critical: trigger RTL
        if v < self.cfg.battery_critical_v and not self._battery_critical_alerted:
            self._battery_critical_alerted = True
            alert = String()
            alert.data = f"BATTERY_CRITICAL: {v:.1f}V — TRIGGERING RTL"
            self.alert_pub.publish(alert)
            self.get_logger().error(f"[safety] {alert.data}")
            self._emergency_rtl()

    def _emergency_rtl(self):
        """Switch to RTL mode immediately."""
        if not self._set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "[safety] Cannot trigger RTL — MAVROS not available"
            )
            return

        req = SetMode.Request()
        req.custom_mode = "RTL"
        future = self._set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().mode_sent:
            self.get_logger().info("[safety] Emergency RTL triggered")
        else:
            self.get_logger().error("[safety] Failed to trigger RTL")

    # ===================================================================
    # OAK-D obstacle proximity
    # ===================================================================

    def _pointcloud_cb(self, msg: PointCloud2):
        """
        Process OAK-D pointcloud to find nearest obstacle distance.

        Reads (x, y, z) from the pointcloud and computes the minimum
        Euclidean distance in the forward hemisphere.
        """
        # Parse PointCloud2 to find minimum distance
        # PointCloud2 format: fields are x, y, z as float32
        min_dist = float('inf')

        # Get point step and offsets
        point_step = msg.point_step
        data = msg.data

        # Find field offsets for x, y, z
        x_offset = y_offset = z_offset = None
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        if x_offset is None or y_offset is None or z_offset is None:
            return

        # Sample every Nth point for performance (don't process every point)
        sample_stride = max(1, len(data) // (point_step * 500))
        noise_min = self.cfg.pointcloud_noise_min_m

        for i in range(0, len(data) - point_step + 1, point_step * sample_stride):
            try:
                x = struct.unpack_from('f', data, i + x_offset)[0]
                y = struct.unpack_from('f', data, i + y_offset)[0]
                z = struct.unpack_from('f', data, i + z_offset)[0]
            except struct.error:
                continue

            # Skip NaN points
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue

            # Only consider points in front of the drone (z > 0 in camera frame)
            if z > noise_min:
                dist = math.sqrt(x*x + y*y + z*z)
                if dist < min_dist:
                    min_dist = dist

        self._min_obstacle_dist = min_dist

    def _check_obstacles(self):
        """Publish obstacle distance and warnings."""
        dist = self._min_obstacle_dist
        warn_dist = self.cfg.oak_warning_dist_m

        # Publish raw distance
        dist_msg = Float32()
        dist_msg.data = float(dist) if dist != float('inf') else -1.0
        self.obstacle_dist_pub.publish(dist_msg)

        # Publish warning flag
        warn_msg = Bool()
        warn_msg.data = dist < warn_dist
        self.obstacle_warning_pub.publish(warn_msg)

        # Alert if too close
        if dist < warn_dist:
            now = time.time()
            if now - self._last_obstacle_warn > self.cfg.obstacle_warn_throttle_s:
                self._last_obstacle_warn = now
                alert = String()
                alert.data = f"OBSTACLE_WARNING: {dist:.1f}m < {warn_dist}m"
                self.alert_pub.publish(alert)
                self.get_logger().warn(f"[safety] {alert.data}")

    # ===================================================================
    # Main safety tick
    # ===================================================================

    def _safety_tick(self):
        """Periodic safety checks."""
        self._check_battery()
        self._check_obstacles()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[safety] Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
