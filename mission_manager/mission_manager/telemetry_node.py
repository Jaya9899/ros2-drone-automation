# telemetry_node.py — Unified telemetry aggregator for the mission
#
# Subscribes to MAVROS and mission topics, publishes a combined JSON
# string on /telemetry/combined at a configurable rate.
#
# Usage:
#   ros2 run mission_manager telemetry_node

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped

from mission_manager.config_loader import load_mission_config


class TelemetryNode(Node):
    def __init__(self):
        super().__init__("telemetry_node")
        self.cfg = load_mission_config(self)

        # QoS profile for high-frequency telemetry / sensors
        # MAVROS often publishes these as BEST_EFFORT
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Telemetry state ──────────────────────────────────────
        self._lat = 0.0
        self._lon = 0.0
        self._alt_gps = 0.0
        self._alt_local = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._battery_v = 0.0
        self._battery_pct = -1.0
        self._mission_state = "UNKNOWN"

        # ── Publisher ────────────────────────────────────────────
        self._pub = self.create_publisher(String, "/telemetry/combined", 10)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._gps_cb, sensor_qos,
        )
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, sensor_qos,
        )
        self.create_subscription(
            Imu, "/mavros/imu/data",
            self._imu_cb, sensor_qos,
        )
        self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local",
            self._vel_cb, sensor_qos,
        )
        self.create_subscription(
            BatteryState, "/mavros/battery",
            self._battery_cb, sensor_qos,
        )
        self.create_subscription(
            String, "/mission/status",
            self._status_cb, 10,
        )

        # ── Publish timer ────────────────────────────────────────
        rate = self.cfg.tick_rate_hz  # reuse tick_rate or dedicated
        try:
            rate = self.get_parameter("publish_rate_hz").value
        except Exception:
            pass
        self.create_timer(1.0 / rate, self._publish_tick)

        self.get_logger().info(
            f"[telemetry] Aggregator started — publishing at {rate} Hz "
            f"on /telemetry/combined"
        )

    # ── Callbacks ────────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._alt_gps = msg.altitude

    def _pose_cb(self, msg: PoseStamped):
        self._alt_local = msg.pose.position.z

    def _imu_cb(self, msg: Imu):
        # Quaternion → Euler (roll, pitch, yaw)
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self._pitch = math.degrees(math.asin(sinp))

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _vel_cb(self, msg: TwistStamped):
        self._vx = msg.twist.linear.x
        self._vy = msg.twist.linear.y
        self._vz = msg.twist.linear.z

    def _battery_cb(self, msg: BatteryState):
        self._battery_v = msg.voltage
        self._battery_pct = msg.percentage  # -1 if unknown

    def _status_cb(self, msg: String):
        self._mission_state = msg.data

    # ── Publish combined telemetry ───────────────────────────────

    def _publish_tick(self):
        speed = math.sqrt(self._vx**2 + self._vy**2 + self._vz**2)
        data = {
            "gps": {
                "lat": round(self._lat, 7),
                "lon": round(self._lon, 7),
                "alt": round(self._alt_gps, 2),
            },
            "local_alt": round(self._alt_local, 2),
            "attitude": {
                "roll": round(self._roll, 1),
                "pitch": round(self._pitch, 1),
                "yaw": round(self._yaw, 1),
            },
            "velocity": {
                "vx": round(self._vx, 2),
                "vy": round(self._vy, 2),
                "vz": round(self._vz, 2),
                "speed": round(speed, 2),
            },
            "battery": {
                "voltage": round(self._battery_v, 2),
                "percentage": round(self._battery_pct, 1),
            },
            "mission_state": self._mission_state,
        }

        msg = String()
        msg.data = json.dumps(data)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[telemetry] Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
