# payload_node.py — ROS2 node for payload delivery mechanism
#
# Provides three Trigger services consumed by the mission_manager:
#   /payload/lower         — begin lowering payload via servo/winch
#   /payload/wait_contact  — block until ground contact is detected
#   /payload/release       — release the payload from the mechanism
#
# Hardware interface:
#   Uses MAVROS /mavros/cmd/command to send MAVLink DO_SET_SERVO commands
#   to control the winch servo and the release servo.
#
# Contact detection:
#   Monitors /mavros/rc/in for servo feedback and motor current.
#   Falls back to time-based heuristic when no sensor data is available.
#
# Usage:
#   ros2 run payload payload_node
#   ros2 run payload payload_node --ros-args -p winch_channel:=9 -p release_channel:=10

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32

from mission_manager.config_loader import load_mission_config


class PayloadNode(Node):
    def __init__(self):
        super().__init__("payload_node")

        # ----- Load config -----
        self.cfg = load_mission_config(self)

        # Convenience aliases for frequently accessed params
        self.winch_channel    = self.cfg.winch_channel
        self.release_channel  = self.cfg.release_channel
        self.winch_pwm_lower  = self.cfg.winch_pwm_lower
        self.winch_pwm_stop   = self.cfg.winch_pwm_stop
        self.release_pwm_open = self.cfg.release_pwm_open
        self.release_pwm_lock = self.cfg.release_pwm_lock
        self.lower_duration   = self.cfg.lower_duration_s
        self.contact_timeout  = self.cfg.contact_timeout_s
        self.release_pause    = self.cfg.release_pause_s

        # ----- Internal state -----
        self.is_lowered  = False
        self.is_released = False

        # ----- Motor current monitoring (for contact detection) -----
        self._motor_current = None
        self.create_subscription(
            Float32, "/payload/motor_current",
            self._motor_current_cb, 10,
        )

        # ----- RC feedback (servo readback via /mavros/rc/in) -----
        self._rc_channels = []
        try:
            from mavros_msgs.msg import RCIn
            self.create_subscription(
                RCIn, "/mavros/rc/in",
                self._rc_in_cb, 10,
            )
        except ImportError:
            pass

        # ----- Publisher for current monitoring telemetry -----
        self._current_pub = self.create_publisher(
            Float32, "/payload/current_draw", 10,
        )

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
                f"(winch ch={self.winch_channel}, release ch={self.release_channel})"
            )
        except ImportError:
            self._has_mavros = False
            self.servo_client = None
            self.get_logger().warn(
                "[payload] mavros_msgs not available — running in STUB mode"
            )

        # ----- Services -----
        self.create_service(Trigger, "/payload/lower",        self._handle_lower)
        self.create_service(Trigger, "/payload/wait_contact", self._handle_wait_contact)
        self.create_service(Trigger, "/payload/release",      self._handle_release)

        # Lock release mechanism on startup
        self._set_servo(self.release_channel, self.release_pwm_lock)

        self.get_logger().info(
            "[payload] Payload node ready — "
            "services: /payload/lower, /payload/wait_contact, /payload/release"
        )

    # -----------------------------------------------------------------------
    # Sensor callbacks
    # -----------------------------------------------------------------------
    def _motor_current_cb(self, msg: Float32):
        """Track motor current draw for stall/contact detection."""
        self._motor_current = msg.data
        # Republish for telemetry
        self._current_pub.publish(msg)

    def _rc_in_cb(self, msg):
        """Track RC channel values for servo feedback."""
        self._rc_channels = list(msg.channels)

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
    # /payload/lower — run the winch to lower the payload
    # -----------------------------------------------------------------------
    def _handle_lower(self, request, response):
        self.get_logger().info("[payload] LOWER requested")

        if self.is_lowered:
            response.success = True
            response.message = "Payload already lowered"
            return response

        # Start the winch motor
        ok = self._set_servo(self.winch_channel, self.winch_pwm_lower)
        if not ok:
            response.success = False
            response.message = "Failed to activate winch servo"
            return response

        # Run the winch for the configured duration
        self.get_logger().info(
            f"[payload] Winch running for {self.lower_duration}s..."
        )
        time.sleep(self.lower_duration)

        # Stop the winch
        self._set_servo(self.winch_channel, self.winch_pwm_stop)

        self.is_lowered = True
        response.success = True
        response.message = f"Payload lowered ({self.lower_duration}s winch run)"
        self.get_logger().info(f"[payload] {response.message}")
        return response

    # -----------------------------------------------------------------------
    # /payload/wait_contact — wait until the payload touches the ground
    # -----------------------------------------------------------------------
    def _handle_wait_contact(self, request, response):
        self.get_logger().info("[payload] WAIT_CONTACT requested")

        if not self.is_lowered:
            response.success = False
            response.message = "Payload not lowered yet — call /payload/lower first"
            return response

        self.get_logger().info(
            f"[payload] Waiting up to {self.contact_timeout}s for ground contact..."
        )

        contact_detected = False
        deadline = time.time() + self.contact_timeout
        check_interval = 0.5  # seconds between sensor polls

        while time.time() < deadline:
            # --- Motor current stall detection ---
            # When the payload touches ground, winch cable goes slack and
            # motor current drops significantly.
            if self._motor_current is not None:
                if self._motor_current < 0.2:  # Amps — cable slack
                    self.get_logger().info(
                        f"[payload] Motor current low ({self._motor_current:.2f}A) "
                        f"— ground contact via current sense"
                    )
                    contact_detected = True
                    break

            # --- RC channel feedback (servo position) ---
            # If the winch servo readback shows stall position
            if self._rc_channels and self.winch_channel < len(self._rc_channels):
                readback = self._rc_channels[self.winch_channel]
                if abs(readback - self.winch_pwm_stop) < 50:
                    self.get_logger().info(
                        f"[payload] Servo readback near stop ({readback}) "
                        f"— possible contact"
                    )
                    # Don't immediately confirm — wait for current sense too
                    pass

            # --- Fallback: time-based heuristic ---
            # If no sensor data is available, assume contact after the
            # lower_duration has elapsed at the configured drop altitude.
            if self._motor_current is None and not self._rc_channels:
                self.get_logger().info(
                    "[payload] No sensor data — assuming contact (time-based)"
                )
                contact_detected = True
                break

            time.sleep(check_interval)

        if contact_detected:
            response.success = True
            response.message = "Ground contact confirmed"
            self.get_logger().info("[payload] Ground contact confirmed")
        else:
            response.success = False
            response.message = f"Contact timeout after {self.contact_timeout}s"
            self.get_logger().warn(f"[payload] {response.message}")

        return response

    # -----------------------------------------------------------------------
    # /payload/release — detach the payload from the mechanism
    # -----------------------------------------------------------------------
    def _handle_release(self, request, response):
        self.get_logger().info("[payload] RELEASE requested")

        if self.is_released:
            response.success = True
            response.message = "Payload already released"
            return response

        if not self.is_lowered:
            self.get_logger().warn(
                "[payload] Release called before lower — proceeding anyway"
            )

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

        # Retract: stop the winch (rewind would be a future enhancement)
        self._set_servo(self.winch_channel, self.winch_pwm_stop)

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
