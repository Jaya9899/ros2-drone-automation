# safety_monitor.py — Background safety ADVISORY node for the mission
#
# Runs regardless of RC or Auto mode. It commands the vehicle in NO way:
#   1. Battery voltage -> /safety/battery_status + /safety/alert. The actual
#      battery failsafe is ArduPilot's (BATT_LOW_VOLT / BATT_CRT_VOLT /
#      BATT_FS_LOW_ACT / BATT_FS_CRT_ACT); mission_node stops commanding on
#      critical so the two never fight.
#   2. Forward obstacle proximity from /avoidance/scan (skyscan_avoidance)
#      -> /safety/obstacle_distance, /safety/obstacle_warning.
#   3. Geofence: logs the configured polygon. The fence itself must be set on
#      the FC (Mission Planner: FENCE_ENABLE/FENCE_TYPE/FENCE_ACTION/
#      FENCE_ALT_MAX + upload the arena polygon and red-zone exclusion
#      polygons). Writing params from here is deliberately not done: MAVROS 2
#      exposes /mavros/param/set as ParamSetV2 and silently-failed writes at
#      boot are worse than a checked configuration.
#
# Usage:
#   ros2 run mission_manager safety_monitor

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import BatteryState, LaserScan

from skyscan_common.config_loader import load_mission_config

# Forward arc (each side of the nose) used for the obstacle warning.
FORWARD_HALF_ARC_RAD = math.radians(35.0)


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor")
        self.cfg = load_mission_config(self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.declare_parameter(
            "geofence_points",
            [12.97100, 77.59400, 12.97100, 77.59500,
             12.97220, 77.59500, 12.97220, 77.59400],
        )
        flat = self.get_parameter("geofence_points").value
        self._geofence_gps = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]

        # ----- Publishers -----
        self.obstacle_dist_pub = self.create_publisher(Float32, "/safety/obstacle_distance", 10)
        self.battery_status_pub = self.create_publisher(String, "/safety/battery_status", 10)
        self.alert_pub = self.create_publisher(String, "/safety/alert", 10)
        self.obstacle_warning_pub = self.create_publisher(Bool, "/safety/obstacle_warning", 10)

        # ----- Subscribers -----
        self.create_subscription(BatteryState, "/mavros/battery", self._battery_cb, sensor_qos)
        # sector_builder publishes /avoidance/scan RELIABLE; a BEST_EFFORT
        # subscriber is compatible with that.
        self.create_subscription(LaserScan, "/avoidance/scan", self._scan_cb, sensor_qos)

        # ----- Internal state -----
        self._battery_voltage = None
        self._battery_alerted = False
        self._battery_critical_alerted = False
        self._min_obstacle_dist = float('inf')
        self._last_scan_t = None
        self._last_obstacle_warn = 0.0

        self._log_geofence()
        self.create_timer(1.0 / self.cfg.safety_check_rate_hz, self._safety_tick)

        self.get_logger().info(
            f"[safety] Monitor started — battery warn/critical: "
            f"{self.cfg.battery_warning_v}/{self.cfg.battery_critical_v} V, "
            f"obstacle warn: {self.cfg.oak_warning_dist_m} m")

    def _log_geofence(self):
        self.get_logger().info(
            "[safety] Expected geofence polygon (must be uploaded on the FC):")
        for i, (lat, lon) in enumerate(self._geofence_gps):
            self.get_logger().info(f"  FENCE_POINT_{i}: lat={lat:.7f}, lon={lon:.7f}")

    # ===================================================================
    # Battery
    # ===================================================================

    def _battery_cb(self, msg: BatteryState):
        self._battery_voltage = msg.voltage

    def _alert(self, text: str, error: bool = False):
        self.alert_pub.publish(String(data=text))
        # Separate call sites: rclpy rejects one call site logging at two
        # different severities ("Logger severity cannot be changed").
        if error:
            self.get_logger().error(f"[safety] {text}")
        else:
            self.get_logger().warn(f"[safety] {text}")

    def _check_battery(self):
        v = self._battery_voltage
        if v is None or v <= 0:
            return

        if v < self.cfg.battery_critical_v:
            status = f"CRITICAL:{v:.1f}V"
        elif v < self.cfg.battery_warning_v:
            status = f"WARNING:{v:.1f}V"
        else:
            status = f"OK:{v:.1f}V"
        self.battery_status_pub.publish(String(data=status))

        if v < self.cfg.battery_warning_v and not self._battery_alerted:
            self._battery_alerted = True
            self._alert(f"BATTERY_WARNING: {v:.1f}V < {self.cfg.battery_warning_v}V")

        if v < self.cfg.battery_critical_v and not self._battery_critical_alerted:
            self._battery_critical_alerted = True
            self._alert(f"BATTERY_CRITICAL: {v:.1f}V < {self.cfg.battery_critical_v}V "
                        "— ArduPilot battery failsafe should be acting", error=True)

    # ===================================================================
    # Obstacle proximity from /avoidance/scan
    # ===================================================================

    def _scan_cb(self, msg: LaserScan):
        best = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= FORWARD_HALF_ARC_RAD and r < best:
                best = r
        self._min_obstacle_dist = best
        self._last_scan_t = time.time()

    def _check_obstacles(self):
        # A stale scan means "unknown", never "clear".
        stale = self._last_scan_t is None or time.time() - self._last_scan_t > 1.0
        dist = float('inf') if stale else self._min_obstacle_dist
        warn_dist = self.cfg.oak_warning_dist_m

        self.obstacle_dist_pub.publish(Float32(data=float(dist) if math.isfinite(dist) else -1.0))
        self.obstacle_warning_pub.publish(Bool(data=bool(dist < warn_dist)))

        if dist < warn_dist:
            now = time.time()
            if now - self._last_obstacle_warn > self.cfg.obstacle_warn_throttle_s:
                self._last_obstacle_warn = now
                self._alert(f"OBSTACLE_WARNING: {dist:.1f}m < {warn_dist}m")

    def _safety_tick(self):
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
        if rclpy.ok():
            rclpy.shutdown()
