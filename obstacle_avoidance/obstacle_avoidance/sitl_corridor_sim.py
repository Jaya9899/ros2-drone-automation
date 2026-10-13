# sitl_corridor_sim.py — SITL stand-in for the OAK-D and the downward camera
# in the corridors. NOT for the vehicle.
#
# Ray-casts the forward depth image (corridor_render) from the SITL vehicle
# pose into the two 5 ft corridors defined by the mission's corridor
# waypoints, with box obstacles, and publishes it where the OAK driver would
# (/oak/stereo/image_raw 32FC1 + camera_info). The REAL sector_builder,
# coverage_guard and corridor_perception nodes then run on it, so path_planner
# is exercised closed-loop. Also publishes /corridor/ground_marker (green over
# the entrance strip, blue over the exit strip) in place of ground_marker_node.
#
# gps_offset_m shifts the world laterally against the EKF (true position =
# EKF + offset, to the LEFT of each corridor): GPS-only flight would hit a
# wall; the camera-based centring must correct it.
#
# Corridor geometry: the first waypoint is mouth_offset_m before the mouth,
# the walls end exit_offset_m before the last waypoint. Obstacles per corridor:
# flat list [s_front, lateral_centre(+left), width, top_height, ...] with s
# measured from the mouth; each box is obstacle_depth_m long.

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix
from std_msgs.msg import Float64, String

from mission_common import mavros_utils as mav
from obstacle_avoidance import geometry as g
from obstacle_avoidance.corridor_render import Camera, Scene, render

_DA = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
M_PER_DEG = 111_320.0


class SitlCorridorSim(Node):
    def __init__(self):
        super().__init__('sitl_corridor_sim')
        dp = self.declare_parameter
        dp('origin_lat', 12.9716)
        dp('origin_lon', 77.5946)
        for i, (x, y) in ((1, ([-3.0, 13.0], [15.0, 15.0])),
                          (2, ([13.0, -3.0], [-5.0, -5.0]))):
            dp(f'corridor{i}_waypoints_x', x, _DA)
            dp(f'corridor{i}_waypoints_y', y, _DA)
            dp(f'corridor{i}_obstacles', [0.0], _DA)
        dp('corridor_width_m', 1.524)
        dp('wall_height_m', 3.0)
        dp('mouth_offset_m', 2.5)
        dp('exit_offset_m', 1.0)
        dp('obstacle_depth_m', 0.4)
        dp('gps_offset_m', 0.0)
        dp('depth_rate_hz', 5.0)
        dp('render_scale', 0.5)        # 320x240: ray casting 640x480 takes ~90 ms
        dp('marker_strip_m', 1.0)
        gp = lambda n: self.get_parameter(n).value          # noqa: E731

        lat0, lon0 = float(gp('origin_lat')), float(gp('origin_lon'))

        def to_gps(x, y):
            return (lat0 + y / M_PER_DEG,
                    lon0 + x / (M_PER_DEG * math.cos(math.radians(lat0))))

        self.corridors = []
        for i in (1, 2):
            xs, ys = gp(f'corridor{i}_waypoints_x'), gp(f'corridor{i}_waypoints_y')
            first, last = to_gps(xs[0], ys[0]), to_gps(xs[-1], ys[-1])
            d_n, d_e = mav.ne_offset(*first, *last)
            L = math.hypot(d_n, d_e)
            mouth = float(gp('mouth_offset_m'))
            length = L - mouth - float(gp('exit_offset_m'))
            ob = list(gp(f'corridor{i}_obstacles'))
            boxes = []
            for k in range(0, len(ob) - 3, 4):
                s, c, w, top = ob[k:k + 4]
                boxes.append((s, s + float(gp('obstacle_depth_m')), c - w / 2, c + w / 2, top))
            self.corridors.append(dict(
                name=f'corridor{i}', first=first, axis=(d_n / L, d_e / L),
                yaw=math.atan2(d_n, d_e), mouth=mouth, length=length,
                scene=Scene(width=float(gp('corridor_width_m')), x_start=0.0,
                            x_end=length, wall_height=float(gp('wall_height_m')),
                            boxes=boxes)))
            self.get_logger().info(
                f'[sitl_corridor_sim] corridor{i}: {length:.1f} m walls, '
                f'{len(boxes)} obstacles {boxes}')
        self.offset = float(gp('gps_offset_m'))
        self.strip = float(gp('marker_strip_m'))
        k = float(gp('render_scale'))
        c = Camera()
        self.cam = Camera(fx=c.fx * k, fy=c.fy * k, cx=c.cx * k, cy=c.cy * k,
                          width=int(c.width * k), height=int(c.height * k))

        self._gps = self._alt = self._yaw = None
        self._pitch = self._roll = 0.0
        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 lambda m: setattr(self, '_gps', (m.latitude, m.longitude)), best)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 lambda m: setattr(self, '_alt', m.data), best)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg',
                                 lambda m: setattr(self, '_yaw', math.radians(90.0 - m.data)), best)
        self.create_subscription(Imu, '/mavros/imu/data', self._on_imu, best)
        self.depth_pub = self.create_publisher(Image, '/oak/stereo/image_raw', best)
        self.info_pub = self.create_publisher(CameraInfo, '/oak/stereo/camera_info', best)
        self.marker_pub = self.create_publisher(String, '/corridor/ground_marker', 10)
        self.create_timer(1.0 / float(gp('depth_rate_hz')), self._tick)

    def _on_imu(self, m):
        q = m.orientation
        self._pitch = g.pitch_from_quaternion(q.x, q.y, q.z, q.w)
        self._roll = g.roll_from_quaternion(q.x, q.y, q.z, q.w)

    def _corridor_pose(self, c):
        """Vehicle (x along from the mouth, y left, yaw rel.) in corridor c,
        with the deliberate EKF offset applied."""
        d_n, d_e = mav.ne_offset(*c['first'], *self._gps)
        a_n, a_e = c['axis']
        x = d_n * a_n + d_e * a_e - c['mouth']
        y = d_n * a_e - d_e * a_n + self.offset
        return x, y, math.atan2(math.sin(self._yaw - c['yaw']), math.cos(self._yaw - c['yaw']))

    def _tick(self):
        if self._gps is None or self._alt is None or self._yaw is None:
            return
        poses = [(c, *self._corridor_pose(c)) for c in self.corridors]
        c, x, y, yaw = min(poses, key=lambda p: abs(p[2]) + max(0.0, -p[1] - 6.0)
                           + max(0.0, p[1] - p[0]['length'] - 6.0))
        near = abs(y) < 6.0 and -8.0 < x < c['length'] + 8.0
        scene = c['scene'] if near else Scene(x_start=1e3, x_end=1e3 + 1.0)
        z = render(scene, self.cam, (x, y, max(self._alt, 0.05)), yaw,
                   self._pitch, self._roll)
        stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(self._info(stamp))
        self.depth_pub.publish(self._image(z, stamp))

        marker = 'none'
        if near and abs(y) < c['scene'].width / 2 + 0.5:
            if -self.strip <= x <= 0.0:
                marker = 'green'
            elif c['length'] <= x <= c['length'] + self.strip:
                marker = 'blue'
        self.marker_pub.publish(String(data=marker))

    def _info(self, stamp):
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, 'oak_forward'
        m.width, m.height = self.cam.width, self.cam.height
        m.k = [self.cam.fx, 0.0, self.cam.cx, 0.0, self.cam.fy, self.cam.cy, 0.0, 0.0, 1.0]
        return m

    def _image(self, z, stamp):
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, 'oak_forward'
        m.height, m.width = z.shape
        m.encoding, m.is_bigendian = '32FC1', 0
        m.step = z.shape[1] * 4
        m.data = np.ascontiguousarray(z, dtype=np.float32).tobytes()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = SitlCorridorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
