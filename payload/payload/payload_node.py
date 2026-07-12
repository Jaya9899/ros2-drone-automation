# payload_node.py — ROS2 node for payload delivery mechanism
#
# Provides one Trigger service consumed by the mission_manager:
#   /payload/release       — release the payload from the mechanism
#
# Hardware interface:
#   Uses MAVROS /mavros/cmd/command to send MAVLink DO_SET_SERVO commands
#   to control the release servo.
#
# Usage:
#   ros2 run payload payload_node
#   ros2 run payload payload_node --ros-args -p release_channel:=10

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger

from mission_manager.config_loader import load_mission_config


class PayloadNode(Node):
    def __init__(self):
        super().__init__("payload_node")

        # ----- Load config -----
        self.cfg = load_mission_config(self)

        # Convenience aliases for frequently accessed params
        self.release_channel  = self.cfg.release_channel
        self.release_pwm_open = self.cfg.release_pwm_open
        self.release_pwm_lock = self.cfg.release_pwm_lock
        self.release_pause    = self.cfg.release_pause_s

        # ----- Internal state -----
        self.is_released = False

        # ----- MAVROS servo command client -----
        try:
            from mavros_msgs.srv import CommandLong
            self._cmd_long_type = CommandLong
            self.servo_client = self.create_client(
                CommandLong, "/mavros/cmd/command"
            )
            self._has_mavros = True
            self.get_logger().info(
                "[payload] MAVROS CommandLong client created "
                f"(release ch={self.release_channel})"
            )
        except ImportError:
            self._has_mavros = False
            self.servo_client = None
            self.get_logger().warn(
                "[payload] mavros_msgs not available — running in STUB mode"
            )

        # ----- Services -----
        self.create_service(Trigger, "/payload/release",      self._handle_release)

        # Lock release mechanism on startup
        self._set_servo(self.release_channel, self.release_pwm_lock)

        self.get_logger().info(
            "[payload] Payload node ready — "
            "services: /payload/release"
        )

    # -----------------------------------------------------------------------
    # Servo helper — sends MAVLink DO_SET_SERVO via MAVROS CommandLong
    # -----------------------------------------------------------------------
    def _set_servo(self, channel: int, pwm: int) -> bool:
        """
        Send a DO_SET_SERVO command through MAVROS.

        MAVLink command 183 (MAV_CMD_DO_SET_SERVO):
          param1 = servo instance/channel number
          param2 = PWM value (µs)
        """
        if not self._has_mavros:
            self.get_logger().info(
                f"[payload] STUB: set_servo(ch={channel}, pwm={pwm})"
            )
            return True

        if not self.servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "[payload] /mavros/cmd/command service not available"
            )
            return False

        req = self._cmd_long_type.Request()
        req.broadcast = False
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.param1 = float(channel)
        req.param2 = float(pwm)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info(
                f"[payload] Servo ch={channel} set to PWM={pwm}"
            )
            return True

        self.get_logger().error(
            f"[payload] Failed to set servo ch={channel} to PWM={pwm}"
        )
        return False

    # -----------------------------------------------------------------------
    # /payload/release — detach the payload from the mechanism
    # -----------------------------------------------------------------------
    def _handle_release(self, request, response):
        self.get_logger().info("[payload] RELEASE requested")

        if self.is_released:
            response.success = True
            response.message = "Payload already released"
            return response

        # Open the release mechanism
        ok = self._set_servo(self.release_channel, self.release_pwm_open)
        if not ok:
            response.success = False
            response.message = "Failed to activate release servo"
            return response

        # Brief pause to let the mechanism fully open
        time.sleep(self.release_pause)

        self.is_released = True
        response.success = True
        response.message = "Payload released"
        self.get_logger().info("[payload] Payload released successfully")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PayloadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[payload] Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
