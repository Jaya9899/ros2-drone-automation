# sitl_qr_simulator.py — Simulates QR detections for SITL testing
#
# Since there is no real camera in SITL, this node watches the drone's
# GPS position and publishes a QR detection on /qr/detection when the
# drone flies within detection_radius_m of a pre-configured QR location.
#
# Usage:
#   ros2 run mission_manager sitl_qr_simulator
#   ros2 run mission_manager sitl_qr_simulator --ros-args \
#       -p qr_location_lat:=12.9718 -p qr_location_lon:=77.5948

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

from mission_manager.config_loader import load_mission_config


class SitlQrSimulator(Node):
    def __init__(self):
        super().__init__("sitl_qr_simulator")
        self.cfg = load_mission_config(self)

        # QoS profile for high-frequency telemetry / sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── QR target parameters ─────────────────────────────────
        self.declare_parameter("qr_location_lat", 12.9718)
        self.declare_parameter("qr_location_lon", 77.5948)
        self.declare_parameter("qr_content", "QR_TARGET_DEMO_ID")
        self.declare_parameter("detection_radius_m", 5.0)

        self.qr_lat    = self.get_parameter("qr_location_lat").value
        self.qr_lon    = self.get_parameter("qr_location_lon").value
        self.qr_content = self.get_parameter("qr_content").value
        self.det_radius = self.get_parameter("detection_radius_m").value

        # ── State ────────────────────────────────────────────────
        self._detected = False  # only fire once

        # ── Publisher ────────────────────────────────────────────
        self._pub = self.create_publisher(String, "/qr/detection", 10)

        # ── GPS subscriber ───────────────────────────────────────
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._gps_cb, sensor_qos,
        )

        self.get_logger().info(
            f"[sitl_qr] Simulating QR '{self.qr_content}' at "
            f"({self.qr_lat:.7f}, {self.qr_lon:.7f}), "
            f"detect within {self.det_radius} m"
        )

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2):
        """Return distance in metres between two GPS points."""
        R = 6_371_000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def _gps_cb(self, msg: NavSatFix):
        if self._detected:
            return

        dist = self._haversine(msg.latitude, msg.longitude,
                               self.qr_lat, self.qr_lon)

        if dist <= self.det_radius:
            self._detected = True
            # Publish in the same format the perception node uses
            detection = f"{self.qr_content}|{self.qr_lat}|{self.qr_lon}"
            out = String()
            out.data = detection
            self._pub.publish(out)
            self.get_logger().info(
                f"[sitl_qr] QR DETECTED at {dist:.1f} m — "
                f"published '{detection}'"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SitlQrSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[sitl_qr] Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
