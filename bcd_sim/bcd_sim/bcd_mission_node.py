#!/usr/bin/env python3
"""
bcd_mission_node.py — ROS 2 node: BCD lawnmower on ArduPilot SITL + MAVROS
Publishes RViz markers (arena, red zones, path, drone trail) and flies
the BCD pattern using MAVROS goto + wait_for_arrival.

Usage (3 terminals):
  T1: cd ~/ardupilot/Tools/autotest && python3 sim_vehicle.py -v ArduCopter -f quad --no-mavproxy -l 12.9716,77.5946,0,0
  T2: source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760
  T3: source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && python3 ~/ros2_ws/src/bcd_sim/bcd_sim/bcd_mission_node.py
"""

import math, time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State as MavState

# ── Arena config (matches mission_params.yaml) ──
ARENA_W, ARENA_H = 40.0, 30.0
ALTITUDE = 5.0
HFOV_DEG, OVERLAP = 62.2, 0.80
ORIGIN_LAT, ORIGIN_LON = 12.9716, 77.5946
RED_ZONES = [
    [(24,12),(36,12),(36,22),(24,22)],
    [(10,5),(15,5),(12.5,10)],
    [(5,20),(10,20),(10,22),(7,22),(7,25),(5,25)],
]
BUFFER_M = 0.5

def local_to_gps(x, y, olat, olon):
    lat = olat + y / 111320.0
    lon = olon + x / (111320.0 * math.cos(math.radians(olat)))
    return lat, lon

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2-lat1), math.radians(lon2-lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

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

class BCDMissionNode(Node):
    def __init__(self):
        super().__init__("bcd_mission_node")
        self.strip_w, self.local_wps = generate_waypoints()
        self.gps_wps = [local_to_gps(x,y,ORIGIN_LAT,ORIGIN_LON) for x,y in self.local_wps]
        self.get_logger().info(f"Generated {len(self.gps_wps)} BCD waypoints, strip={self.strip_w:.2f}m")

        sq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)

        # Publishers
        self.path_pub = self.create_publisher(Path, "/bcd/path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/bcd/markers", 10)
        self.trail_pub = self.create_publisher(Path, "/bcd/trail", 10)

        # Subscribers
        self.mav_state = None
        self.cur_gps = None
        self.create_subscription(MavState, "/mavros/state", self._state_cb, sq)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._gps_cb, sq)

        # Service clients
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        # GPS setpoint publisher
        try:
            from mavros_msgs.msg import GlobalPositionTarget
            self.gpt_cls = GlobalPositionTarget
            self.goto_pub = self.create_publisher(GlobalPositionTarget, "/mavros/setpoint_position/global", 10)
        except ImportError:
            self.gpt_cls = None; self.goto_pub = None

        # Trail tracking
        self.trail_points = []
        self.current_wp_idx = 0

        # Timers
        self.create_timer(1.0, self._publish_viz)

        # Mission thread
        threading.Thread(target=self._run_mission, daemon=True).start()

    def _state_cb(self, msg): self.mav_state = msg
    def _gps_cb(self, msg): self.cur_gps = (msg.latitude, msg.longitude)

    # ── MAVROS helpers ──
    def _call_srv(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Service not available"); return None
        fut = cli.call_async(req)
        deadline = time.time()+timeout
        while not fut.done() and time.time()<deadline: time.sleep(0.05)
        return fut.result()

    def _set_mode(self, mode):
        req = SetMode.Request(); req.custom_mode = mode
        r = self._call_srv(self.mode_cli, req)
        ok = r and r.mode_sent
        self.get_logger().info(f"Set mode {mode}: {'OK' if ok else 'FAIL'}")
        return ok

    def _arm(self):
        req = CommandBool.Request(); req.value = True
        r = self._call_srv(self.arm_cli, req)
        ok = r and r.success
        self.get_logger().info(f"Arm: {'OK' if ok else 'FAIL'}")
        return ok

    def _takeoff(self, alt):
        req = CommandTOL.Request(); req.altitude = float(alt)
        r = self._call_srv(self.takeoff_cli, req)
        ok = r and r.success
        self.get_logger().info(f"Takeoff {alt}m: {'OK' if ok else 'FAIL'}")
        return ok

    def _goto(self, lat, lon, alt):
        if not self.goto_pub or not self.gpt_cls: return
        msg = self.gpt_cls()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = 6  # FRAME_GLOBAL_REL_ALT
        msg.type_mask = 0b0000_1111_1111_1000
        msg.latitude, msg.longitude, msg.altitude = lat, lon, alt
        for _ in range(20):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.goto_pub.publish(msg); time.sleep(0.1)

    def _wait_arrival(self, lat, lon, tol=2.0, timeout=60.0):
        deadline = time.time()+timeout
        while time.time()<deadline:
            if self.cur_gps:
                d = haversine(self.cur_gps[0], self.cur_gps[1], lat, lon)
                if d <= tol:
                    self.get_logger().info(f"  Arrived ({d:.1f}m)")
                    return True
                if self.cur_gps not in self.trail_points:
                    self.trail_points.append(self.cur_gps)
            time.sleep(0.25)
        self.get_logger().warn(f"  Timeout reaching waypoint")
        return False

    # ── Mission sequence ──
    def _run_mission(self):
        self.get_logger().info("Waiting for MAVROS...")
        while self.mav_state is None or not self.mav_state.connected:
            if not rclpy.ok(): return
            time.sleep(0.5)
        self.get_logger().info("MAVROS connected, waiting EKF...")
        while self.mav_state.system_status != 3:
            if not rclpy.ok(): return
            time.sleep(0.5)

        self.get_logger().info("FCU ready. Starting BCD mission.")
        self._set_mode("GUIDED")
        time.sleep(1)
        self._arm()
        time.sleep(1)
        self._takeoff(ALTITUDE)
        time.sleep(8)  # wait for climb

        for i, (lat, lon) in enumerate(self.gps_wps):
            self.current_wp_idx = i
            lx, ly = self.local_wps[i]
            self.get_logger().info(f"WP {i+1}/{len(self.gps_wps)}: local=({lx:.1f},{ly:.1f})")
            self._goto(lat, lon, ALTITUDE)
            self._wait_arrival(lat, lon, tol=2.0, timeout=30.0)

        self.get_logger().info("BCD complete! Returning to launch.")
        self._set_mode("RTL")

    # ── RViz visualization ──
    def _publish_viz(self):
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # Arena boundary
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

        # Drone position
        if self.cur_gps:
            dx = (self.cur_gps[1]-ORIGIN_LON)*111320*math.cos(math.radians(ORIGIN_LAT))
            dy = (self.cur_gps[0]-ORIGIN_LAT)*111320
            m = Marker(); m.header=Header(frame_id="map",stamp=now)
            m.ns="drone"; m.id=0; m.type=Marker.SPHERE; m.action=Marker.ADD
            m.pose.position=Point(x=dx,y=dy,z=ALTITUDE)
            m.scale.x=1.5; m.scale.y=1.5; m.scale.z=0.5
            m.color=ColorRGBA(r=0.0,g=0.9,b=0.46,a=1.0)
            ma.markers.append(m)

        # Current waypoint target
        if self.current_wp_idx < len(self.local_wps):
            tx,ty = self.local_wps[self.current_wp_idx]
            m = Marker(); m.header=Header(frame_id="map",stamp=now)
            m.ns="target"; m.id=0; m.type=Marker.CYLINDER; m.action=Marker.ADD
            m.pose.position=Point(x=float(tx),y=float(ty),z=ALTITUDE)
            m.scale.x=1.0; m.scale.y=1.0; m.scale.z=0.3
            m.color=ColorRGBA(r=1.0,g=0.92,b=0.0,a=0.8)
            ma.markers.append(m)

        # Progress text
        m = Marker(); m.header=Header(frame_id="map",stamp=now)
        m.ns="text"; m.id=0; m.type=Marker.TEXT_VIEW_FACING; m.action=Marker.ADD
        m.pose.position=Point(x=ARENA_W/2,y=ARENA_H+3.0,z=0.0)
        m.scale.z=1.5
        m.color=ColorRGBA(r=0.9,g=0.9,b=0.9,a=1.0)
        m.text=f"BCD Mission: WP {self.current_wp_idx+1}/{len(self.gps_wps)}"
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
        for lat,lon in self.trail_points:
            dx=(lon-ORIGIN_LON)*111320*math.cos(math.radians(ORIGIN_LAT))
            dy=(lat-ORIGIN_LAT)*111320
            p = PoseStamped(); p.header=trail.header
            p.pose.position=Point(x=dx,y=dy,z=ALTITUDE)
            p.pose.orientation.w=1.0; trail.poses.append(p)
        self.trail_pub.publish(trail)

def main():
    rclpy.init()
    node = BCDMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
