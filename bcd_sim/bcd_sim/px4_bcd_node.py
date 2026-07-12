#!/usr/bin/env python3
"""
px4_bcd_node.py — ROS 2 node for BCD lawnmower on PX4 SITL + Gazebo
Flies the drone using PX4 OFFBOARD mode and local setpoints, and
publishes RViz markers for visualization.
"""

import math, time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State as MavState

# ── Arena config ──
ARENA_W, ARENA_H = 40.0, 30.0
ALTITUDE = 5.0
HFOV_DEG, OVERLAP = 62.2, 0.80
RED_ZONES = [
    [(24,12),(36,12),(36,22),(24,22)],
    [(10,5),(15,5),(12.5,10)],
    [(5,20),(10,20),(10,22),(7,22),(7,25),(5,25)],
]
BUFFER_M = 0.5

def generate_waypoints():
    try:
        from shapely.geometry import Polygon
        arena = Polygon([(0,0),(ARENA_W,0),(ARENA_W,ARENA_H),(0,ARENA_H)])
        flyable = arena
        for rz in RED_ZONES:
            if len(rz)>=3:
                flyable = flyable.difference(Polygon(rz).buffer(BUFFER_M))
        sw = 2*ALTITUDE*math.tan(math.radians(HFOV_DEG)/2)*OVERLAP
        wps, y, ltr = [], flyable.bounds[1]+sw/2, True
        while y <= flyable.bounds[3]:
            slab = Polygon([(-1,y-sw/2),(ARENA_W+1,y-sw/2),(ARENA_W+1,y+sw/2),(-1,y+sw/2)])
            inter = flyable.intersection(slab)
            if not inter.is_empty:
                geoms = inter.geoms if inter.geom_type=="MultiPolygon" else [inter]
                for g in geoms:
                    try:
                        xs = list(g.exterior.xy[0])
                    except: continue
                    if ltr: wps+=[(min(xs),y),(max(xs),y)]
                    else:   wps+=[(max(xs),y),(min(xs),y)]
            ltr = not ltr; y += sw
        return sw, [(0.0,15.0)]+wps
    except ImportError:
        sw = 2*ALTITUDE*math.tan(math.radians(HFOV_DEG)/2)*OVERLAP
        wps, y, ltr = [], sw/2, True
        while y <= ARENA_H:
            if ltr: wps+=[(0,y),(ARENA_W,y)]
            else:   wps+=[(ARENA_W,y),(0,y)]
            ltr = not ltr; y += sw
        return sw, [(0.0,15.0)]+wps

class PX4BCDNode(Node):
    def __init__(self):
        super().__init__("px4_bcd_node")
        self.strip_w, self.local_wps = generate_waypoints()
        self.get_logger().info(f"Generated {len(self.local_wps)} BCD waypoints")

        # QoS
        sq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)

        # Publishers
        self.path_pub = self.create_publisher(Path, "/bcd/path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/bcd/markers", 10)
        self.trail_pub = self.create_publisher(Path, "/bcd/trail", 10)
        
        # MAVROS publishers
        self.setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)

        # Subscribers
        self.mav_state = None
        self.cur_pos = None
        self.create_subscription(MavState, "/mavros/state", self._state_cb, sq)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._pos_cb, sq)

        # Service clients
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.current_wp_idx = 0
        self.trail_points = []
        
        # Target holding
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.z = 0.0

        # Timers
        self.create_timer(0.05, self._stream_setpoints)  # 20Hz required for OFFBOARD
        self.create_timer(1.0, self._publish_viz)

        threading.Thread(target=self._run_mission, daemon=True).start()

    def _state_cb(self, msg): self.mav_state = msg
    def _pos_cb(self, msg): 
        self.cur_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _stream_setpoints(self):
        # PX4 OFFBOARD requires continuous setpoint stream
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.target_pose.pose.position.x
        msg.pose.position.y = self.target_pose.pose.position.y
        msg.pose.position.z = self.target_pose.pose.position.z
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def _call_srv(self, cli, req, timeout=5.0):
        if not cli.wait_for_service(timeout_sec=timeout): return None
        fut = cli.call_async(req)
        deadline = time.time()+timeout
        while not fut.done() and time.time()<deadline: time.sleep(0.05)
        return fut.result()

    def _set_mode(self, mode):
        req = SetMode.Request(); req.custom_mode = mode
        r = self._call_srv(self.mode_cli, req)
        return r and r.mode_sent

    def _arm(self):
        req = CommandBool.Request(); req.value = True
        r = self._call_srv(self.arm_cli, req)
        return r and r.success

    def _wait_arrival(self, tx, ty, tz, tol=1.0, timeout=30.0):
        deadline = time.time()+timeout
        while time.time()<deadline:
            if self.cur_pos:
                cx, cy, cz = self.cur_pos
                dist = math.hypot(cx-tx, cy-ty)
                if dist <= tol and abs(cz-tz) <= tol:
                    return True
                
                # Record trail (2D distance filter)
                if not self.trail_points or math.hypot(cx-self.trail_points[-1][0], cy-self.trail_points[-1][1]) > 0.5:
                    self.trail_points.append((cx, cy))
            time.sleep(0.1)
        return False

    def _run_mission(self):
        self.get_logger().info("Waiting for MAVROS...")
        while self.mav_state is None or not self.mav_state.connected:
            time.sleep(0.5)

        # Wait for local position
        self.get_logger().info("Waiting for local position...")
        while self.cur_pos is None:
            time.sleep(0.5)

        # Hold current position at altitude 0 initially
        self.target_pose.pose.position.x = self.cur_pos[0]
        self.target_pose.pose.position.y = self.cur_pos[1]
        self.target_pose.pose.position.z = self.cur_pos[2]

        self.get_logger().info("Streaming setpoints for 2s before OFFBOARD...")
        time.sleep(2.0)

        self.get_logger().info("Switching to OFFBOARD mode...")
        for _ in range(5):
            if self._set_mode("OFFBOARD"): break
            time.sleep(1)
        
        self.get_logger().info("Arming...")
        for _ in range(5):
            if self._arm(): break
            time.sleep(1)

        # Takeoff to ALTITUDE
        self.get_logger().info(f"Taking off to {ALTITUDE}m...")
        self.target_pose.pose.position.z = ALTITUDE
        self._wait_arrival(self.cur_pos[0], self.cur_pos[1], ALTITUDE, tol=0.5, timeout=15.0)

        # Fly waypoints
        for i, (tx, ty) in enumerate(self.local_wps):
            self.current_wp_idx = i
            self.get_logger().info(f"WP {i+1}/{len(self.local_wps)}: ({tx:.1f}, {ty:.1f})")
            
            self.target_pose.pose.position.x = tx
            self.target_pose.pose.position.y = ty
            self.target_pose.pose.position.z = ALTITUDE
            
            arrived = self._wait_arrival(tx, ty, ALTITUDE, tol=1.5, timeout=40.0)
            if not arrived:
                self.get_logger().warn(f"WP {i+1} timeout, continuing...")

        self.get_logger().info("BCD complete! Landing...")
        self._set_mode("AUTO.LAND")

    # ── RViz visualization ──
    def _publish_viz(self):
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # Arena
        m = Marker(); m.header=Header(frame_id="map",stamp=now)
        m.ns="arena"; m.id=0; m.type=Marker.LINE_STRIP; m.action=Marker.ADD
        m.scale.x=0.3; m.color=ColorRGBA(r=0.25,g=0.72,b=0.31,a=1.0)
        for x,y in [(0,0),(ARENA_W,0),(ARENA_W,ARENA_H),(0,ARENA_H),(0,0)]:
            m.points.append(Point(x=float(x),y=float(y),z=0.0))
        ma.markers.append(m)

        # Red zones
        for zi, rz in enumerate(RED_ZONES):
            m = Marker(); m.header=Header(frame_id="map",stamp=now)
            m.ns="redzone"; m.id=zi; m.type=Marker.LINE_STRIP; m.action=Marker.ADD
            m.scale.x=0.2; m.color=ColorRGBA(r=1.0,g=0.2,b=0.2,a=0.9)
            for x,y in list(rz)+[rz[0]]:
                m.points.append(Point(x=float(x),y=float(y),z=0.0))
            ma.markers.append(m)

        # Target
        if self.current_wp_idx < len(self.local_wps):
            tx,ty = self.local_wps[self.current_wp_idx]
            m = Marker(); m.header=Header(frame_id="map",stamp=now)
            m.ns="target"; m.id=0; m.type=Marker.CYLINDER; m.action=Marker.ADD
            m.pose.position=Point(x=float(tx),y=float(ty),z=ALTITUDE)
            m.scale.x=1.0; m.scale.y=1.0; m.scale.z=0.3
            m.color=ColorRGBA(r=1.0,g=0.92,b=0.0,a=0.8)
            ma.markers.append(m)

        self.marker_pub.publish(ma)

        # Planned path
        path = Path(); path.header=Header(frame_id="map",stamp=now)
        for x,y in self.local_wps:
            p = PoseStamped(); p.header=path.header
            p.pose.position=Point(x=float(x),y=float(y),z=ALTITUDE)
            p.pose.orientation.w=1.0; path.poses.append(p)
        self.path_pub.publish(path)

        # Trail
        trail = Path(); trail.header=Header(frame_id="map",stamp=now)
        for tx, ty in self.trail_points:
            p = PoseStamped(); p.header=trail.header
            p.pose.position=Point(x=tx,y=ty,z=ALTITUDE)
            p.pose.orientation.w=1.0; trail.poses.append(p)
        self.trail_pub.publish(trail)

def main():
    rclpy.init()
    node = PX4BCDNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
