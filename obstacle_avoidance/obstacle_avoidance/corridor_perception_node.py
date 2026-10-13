# corridor_perception_node.py — corridor walls, centring error, obstacles,
# entrance and exit from the forward OAK-D depth.
#
# Runs beside sector_builder_node on the same depth topic. sector_builder feeds
# ArduPilot's proximity map (OBSTACLE_DISTANCE; used by simple avoidance when
# the pilot flies LOITER). This node measures the corridor for path_planner,
# which keeps the vehicle centred and picks the side (or height) to pass an
# obstacle. It only publishes measurements; it commands nothing.
#
#   in : ~/depth, ~/camera_info   (remapped to the OAK driver, like the builder)
#        /mavros/imu/data         (pitch + roll; the slice tilts with the vehicle)
#        /mavros/global_position/rel_alt (ground cut, and cam_alt in the output)
#        /mavros/global_position/compass_hdg (cam_yaw in the output)
#   out: /corridor/perception     (Float32MultiArray, layout in
#                                  mission_common/corridor_msg.py)
#
# Processing runs on a timer at rate_hz on the newest frame, so a slow Pi 5
# frame drops stale depth instead of queueing it. A message is published for
# every processed frame, walls = 0 when no corridor is in view, so the consumer
# can tell "no corridor" (message, walls 0) from "node dead" (no messages).
#
# Geometry in corridor_geometry.py (tested against ray-cast depth in
# test/test_corridor_geometry.py).

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Float32MultiArray, Float64

from mission_common import corridor_msg
from obstacle_avoidance import corridor_geometry as cg
from obstacle_avoidance import geometry as g
from obstacle_avoidance.depth_image import DepthDecodeError, decode_depth

TOPIC = '/corridor/perception'


class CorridorPerceptionNode(Node):

    def __init__(self):
        super().__init__('corridor_perception_node')
        d = cg.CorridorParams()
        self.declare_parameter('corridor_rate_hz', 5.0)
        self.declare_parameter('corridor_stride_px', 4)
        self.declare_parameter('camera_pitch_offset_deg', 0.0)
        self.declare_parameter('pitch_compensation', True)
        self.declare_parameter('corridor_width_m', d.width_m)
        self.declare_parameter('corridor_width_tol_m', d.width_tol_m)
        self.declare_parameter('corridor_slice_lo_m', d.slice_lo_m)
        self.declare_parameter('corridor_slice_hi_m', d.slice_hi_m)
        self.declare_parameter('corridor_max_range_m', d.max_range_m)
        self.declare_parameter('corridor_obstacle_range_m', d.obstacle_range_m)
        gp = lambda n: self.get_parameter(n).value          # noqa: E731
        self.params = cg.CorridorParams(
            width_m=float(gp('corridor_width_m')),
            width_tol_m=float(gp('corridor_width_tol_m')),
            slice_lo_m=float(gp('corridor_slice_lo_m')),
            slice_hi_m=float(gp('corridor_slice_hi_m')),
            max_range_m=float(gp('corridor_max_range_m')),
            obstacle_range_m=float(gp('corridor_obstacle_range_m')),
        )
        self.stride = int(gp('corridor_stride_px'))
        self.pitch_offset = math.radians(float(gp('camera_pitch_offset_deg')))
        self.pitch_compensation = bool(gp('pitch_compensation'))

        self._K = None              # (fx, fy, cx, cy, width)
        self._fov = None
        self._depth_msg = None      # (newest unprocessed frame, alt at arrival)
        self._pitch = self._roll = 0.0
        self._pitch_stamp_s = None
        self._alt = None            # (rel_alt m, stamp s)
        self._yaw = None            # (ENU yaw rad, stamp s)
        self._logged_error = False

        self.pub = self.create_publisher(
            Float32MultiArray, TOPIC,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=5))
        self.create_subscription(Image, '~/depth', self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '~/camera_info', self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self._on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            lambda m: setattr(self, '_alt', (float(m.data), self._now_s())),
            qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg',
            lambda m: setattr(self, '_yaw', (math.radians(90.0 - float(m.data)),
                                             self._now_s())),
            qos_profile_sensor_data)
        self.create_timer(1.0 / float(gp('corridor_rate_hz')), self._on_timer)
        self.get_logger().info(
            f'corridor_perception_node up: width {self.params.width_m:.3f} m, '
            f'slice {self.params.slice_lo_m:+.2f}..{self.params.slice_hi_m:+.2f} m, '
            f'range {self.params.max_range_m:.1f} m -> {TOPIC}')

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_info(self, msg):
        if self._K is not None or msg.k[0] == 0.0 or msg.k[4] == 0.0:
            return                                   # latch the first valid
        self._K = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]),
                   float(msg.k[5]), int(msg.width))
        self._fov = cg.view_limits(self._K[0], self._K[1], self._K[2],
                                   self._K[3], self._K[4])
        self.get_logger().info(
            'Latched intrinsics fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} '
            'W={}; HFOV L/R {:.1f}/{:.1f} deg'.format(
                *self._K, math.degrees(self._fov[0]), math.degrees(self._fov[1])))

    def _on_imu(self, msg):
        q = msg.orientation
        self._pitch = g.pitch_from_quaternion(q.x, q.y, q.z, q.w)
        self._roll = g.roll_from_quaternion(q.x, q.y, q.z, q.w)
        self._pitch_stamp_s = self._now_s()

    def _fresh(self, item):
        if item is not None and self._now_s() - item[1] < 1.0:
            return item[0]
        return float('nan')

    def _on_depth(self, msg):
        # Altitude and attitude are latched with the frame: the vehicle keeps
        # moving while the frame waits for the timer.
        self._depth_msg = (msg, self._fresh(self._alt), self._fresh(self._yaw),
                           self._pitch, self._roll, self._pitch_stamp_s)

    def _on_timer(self):
        item, self._depth_msg = self._depth_msg, None
        if item is None or self._K is None:
            return
        msg, alt, yaw, pitch_raw, roll_raw, att_stamp = item
        try:
            z = decode_depth(msg)
        except DepthDecodeError as e:
            if not self._logged_error:
                self.get_logger().error(f'{e}')
                self._logged_error = True
            return
        fresh = att_stamp is not None and self._now_s() - att_stamp < 0.5
        use = self.pitch_compensation and fresh
        pitch = (pitch_raw - self.pitch_offset) if use else 0.0
        roll = roll_raw if use else 0.0
        if self.pitch_compensation and not fresh:
            self.get_logger().warn(
                'no fresh /mavros/imu/data — corridor fit assumes level camera',
                throttle_duration_sec=5.0)
        fx, fy, cx, cy, _ = self._K
        x, y, h = cg.level_points(z, fx, fy, cx, cy, pitch, self.stride, roll)
        left, right, up = self._fov
        # Height above ground cuts the ground out of the slice (flat ground at
        # home level assumed; ignored if unknown or implausibly low).
        hag = alt if alt > 0.3 else None            # NaN > 0.3 is False
        r = cg.fit_corridor(x, y, h, self.params, fov=(left, right, up + pitch),
                            height_above_ground=hag)
        r['slice_lo'] = self.params.slice_lo_m
        r['cam_alt'] = alt
        r['cam_yaw'] = yaw
        self.pub.publish(corridor_msg.encode(r))
        if r['walls']:
            self.get_logger().info(
                f"walls={r['walls']} L={r['wall_left']:+.2f} R={r['wall_right']:+.2f} "
                f"hdg={math.degrees(r['heading_err']):+.1f}deg "
                f"obs={r['obs_dist']:.2f}m [{r['obs_right']:+.2f},{r['obs_left']:+.2f}] "
                f"top={r['obs_top']:+.2f} mouth={r['mouth_dist']:.2f} "
                f"end={r['walls_end']:.2f}",
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
