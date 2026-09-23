import math
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Header


class VehicleInterface:
    """Minimal vehicle abstraction built on top of MAVROS and ROS2."""

    def __init__(self, node: Optional[Node] = None, namespace: str = "/mavros") -> None:
        self.node = node
        self.namespace = namespace.rstrip("/")
        self._last_state: Optional[State] = None
        self._last_position: Optional[Tuple[float, float]] = None
        self._last_altitude: Optional[float] = None
        self._last_heading: Optional[float] = None
        self._sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._state_sub = None
        self._gps_sub = None
        self._pose_sub = None
        self._heading_sub = None
        self._goto_pub = None
        self._vel_pub = None

    def _ensure_node(self) -> Node:
        if self.node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node("vehicle_interface")
        return self.node

    def _ensure_subscriptions(self) -> None:
        if self.node is None:
            self._ensure_node()

        if self._state_sub is None:
            self._state_sub = self.node.create_subscription(
                State,
                self.namespace + "/state",
                self._state_cb,
                self._sensor_qos,
            )
        if self._gps_sub is None:
            self._gps_sub = self.node.create_subscription(
                NavSatFix,
                self.namespace + "/global_position/global",
                self._gps_cb,
                self._sensor_qos,
            )
        if self._pose_sub is None:
            self._pose_sub = self.node.create_subscription(
                PoseStamped,
                self.namespace + "/local_position/pose",
                self._pose_cb,
                self._sensor_qos,
            )
        if self._heading_sub is None:
            self._heading_sub = self.node.create_subscription(
                Float32,
                self.namespace + "/global_position/compass_hdg",
                self._heading_cb,
                self._sensor_qos,
            )

    def _state_cb(self, msg: State) -> None:
        self._last_state = msg

    def _gps_cb(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:
            self._last_position = (float(msg.latitude), float(msg.longitude))

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._last_altitude = float(msg.pose.position.z)

    def _heading_cb(self, msg: Float32) -> None:
        self._last_heading = float(msg.data)

    def _call_service(self, srv_type, srv_name: str, request, timeout: float = 10.0):
        client = self._ensure_node().create_client(srv_type, srv_name)
        if not client.wait_for_service(timeout_sec=timeout):
            self._ensure_node().get_logger().error(
                f"Service {srv_name} not available after {timeout}s"
            )
            return None

        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if future.result() is None:
            self._ensure_node().get_logger().error(
                f"Service call to {srv_name} failed (no result)"
            )
            return None
        return future.result()

    def connect(self, timeout: float = 30.0) -> bool:
        """Wait until MAVROS reports a valid FCU connection."""
        self._ensure_node()
        self._ensure_subscriptions()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._last_state is not None and self._last_state.connected:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return bool(self._last_state is not None and self._last_state.connected)

    def arm(self, timeout: float = 10.0) -> bool:
        request = CommandBool.Request()
        request.value = True
        result = self._call_service(CommandBool, self.namespace + "/cmd/arming", request, timeout)
        if result and result.success:
            self._ensure_node().get_logger().info("[vehicle_interface] Vehicle armed")
            return True
        self._ensure_node().get_logger().error("[vehicle_interface] Arming failed")
        return False

    def disarm(self, timeout: float = 10.0) -> bool:
        request = CommandBool.Request()
        request.value = False
        result = self._call_service(CommandBool, self.namespace + "/cmd/arming", request, timeout)
        if result and result.success:
            self._ensure_node().get_logger().info("[vehicle_interface] Vehicle disarmed")
            return True
        self._ensure_node().get_logger().error("[vehicle_interface] Disarm failed")
        return False

    def set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        request = SetMode.Request()
        request.custom_mode = mode
        result = self._call_service(SetMode, self.namespace + "/set_mode", request, timeout)
        if result and result.mode_sent:
            self._ensure_node().get_logger().info(f"[vehicle_interface] Mode set to {mode}")
            return True
        self._ensure_node().get_logger().error(f"[vehicle_interface] Failed to set mode {mode}")
        return False

    def takeoff(self, altitude_m: float = 2.0, timeout: float = 15.0) -> bool:
        request = CommandTOL.Request()
        request.altitude = float(altitude_m)
        request.latitude = 0.0
        request.longitude = 0.0
        request.min_pitch = 0.0
        request.yaw = 0.0
        result = self._call_service(CommandTOL, self.namespace + "/cmd/takeoff", request, timeout)
        if result and result.success:
            self._ensure_node().get_logger().info(
                f"[vehicle_interface] Takeoff command accepted ({altitude_m} m)"
            )
            return True
        self._ensure_node().get_logger().error("[vehicle_interface] Takeoff command rejected")
        return False

    def land(self, timeout: float = 10.0) -> bool:
        request = CommandTOL.Request()
        request.altitude = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.min_pitch = 0.0
        request.yaw = 0.0
        result = self._call_service(CommandTOL, self.namespace + "/cmd/land", request, timeout)
        if result and result.success:
            self._ensure_node().get_logger().info("[vehicle_interface] Land command accepted")
            return True
        self._ensure_node().get_logger().warn(
            "[vehicle_interface] /cmd/land failed, falling back to LAND mode"
        )
        return self.set_mode("LAND", timeout=timeout)

    def goto_position(self, lat: float, lon: float, altitude_m: float, publish_count: int = 20) -> bool:
        if self._goto_pub is None:
            self._ensure_node()
            self._goto_pub = self.node.create_publisher(
                GlobalPositionTarget,
                self.namespace + "/setpoint_raw/global",
                10,
            )

        msg = GlobalPositionTarget()
        msg.header = Header()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX
            | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ
            | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY
            | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(altitude_m)

        for _ in range(max(1, int(publish_count))):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self._goto_pub.publish(msg)
            time.sleep(0.1)

        return True

    def set_velocity(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0, yaw_rate: float = 0.0) -> bool:
        self._ensure_node()
        if self._vel_pub is None:
            self._vel_pub = self.node.create_publisher(
                TwistStamped,
                self.namespace + "/setpoint_velocity/cmd_vel",
                10,
            )

        msg = TwistStamped()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(yaw_rate)
        self._vel_pub.publish(msg)
        return True

    def get_position(self, timeout: float = 5.0) -> Optional[Tuple[float, float]]:
        self._ensure_node()
        self._ensure_subscriptions()
        deadline = time.monotonic() + timeout
        while self._last_position is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._last_position

    def get_altitude(self, timeout: float = 5.0) -> Optional[float]:
        self._ensure_node()
        self._ensure_subscriptions()
        deadline = time.monotonic() + timeout
        while self._last_altitude is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._last_altitude

    def get_heading(self, timeout: float = 5.0) -> Optional[float]:
        self._ensure_node()
        self._ensure_subscriptions()
        deadline = time.monotonic() + timeout
        while self._last_heading is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._last_heading

    def get_state(self) -> Dict[str, Any]:
        self._ensure_node()
        self._ensure_subscriptions()
        state = self._last_state
        return {
            "connected": bool(state.connected) if state is not None else False,
            "armed": bool(state.armed) if state is not None else False,
            "guided": bool(state.guided) if state is not None else False,
            "mode": state.mode if state is not None else "UNKNOWN",
            "system_status": int(state.system_status) if state is not None else -1,
            "position": self._last_position,
            "altitude": self._last_altitude,
            "heading": self._last_heading,
        }


__all__ = ["VehicleInterface"]
