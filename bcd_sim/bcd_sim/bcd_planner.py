#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from shapely.geometry import Polygon, MultiPolygon, box
from shapely.ops import unary_union
import numpy as np

class BCDPlanner(Node):
    def __init__(self):
        super().__init__('bcd_planner')

        # Publishers
        self.path_pub = self.create_publisher(Path, '/bcd/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/bcd/markers', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/bcd/next_waypoint', 10)

        # Arena definition (metres, matches your SDF world)
        self.arena = Polygon([
            (-9.8, -9.8),
            ( 9.8, -9.8),
            ( 9.8,  9.8),
            (-9.8,  9.8)
        ])

        # Red zones — must match SDF poses + dimensions
        # Format: (centre_x, centre_y, half_width, half_height)
        self.red_zones = [
            box(-4, 1, -2, 3),    # red_zone_1: centre(-3,2), size 2x2
            box( 2, -4,  4, -2),  # red_zone_2: centre(3,-3), size 2x2
        ]

        # BCD parameters
        self.sweep_altitude = 2.5   # metres AGL
        self.sweep_spacing  = 1.0   # metres between lawnmower passes
        self.waypoint_acceptance_radius = 0.3  # metres

        # State
        self.waypoints = []
        self.current_waypoint_idx = 0

        # Run BCD on startup
        self.waypoints = self.run_bcd()
        self.publish_path()
        self.publish_markers()

        # Timer to publish next waypoint
        self.create_timer(0.5, self.publish_next_waypoint)

        self.get_logger().info(
            f'BCD complete. Generated {len(self.waypoints)} waypoints.'
        )

    # ------------------------------------------------------------------ #
    #  STEP 1 — subtract red zones from arena                             #
    # ------------------------------------------------------------------ #
    def compute_free_space(self):
        obstacle_union = unary_union(self.red_zones)
        # Add safety buffer around each red zone
        buffered = obstacle_union.buffer(0.5)
        free = self.arena.difference(buffered)
        return free

    # ------------------------------------------------------------------ #
    #  STEP 2 — BCD decomposition                                         #
    #  Sweep vertical slices across X axis, find connected free segments  #
    #  in each slice, group into trapezoidal cells                        #
    # ------------------------------------------------------------------ #
    def run_bcd(self):
        free_space = self.compute_free_space()
        bounds = self.arena.bounds  # (minx, miny, maxx, maxy)
        minx, miny, maxx, maxy = bounds

        cells = []
        x = minx
        slice_width = 0.05  # thin vertical slice for event detection

        prev_segments = []
        current_cells = {}  # segment_id -> cell being built

        while x < maxx:
            # Vertical slice at position x
            slice_geom = box(x, miny, x + slice_width, maxy)
            intersection = free_space.intersection(slice_geom)

            # Extract Y segments from intersection
            segments = self.extract_y_segments(intersection, miny, maxy)

            # Detect events: new segments = cell open, disappeared = cell close
            new_ids = set(range(len(segments)))

            # Close cells whose segments disappeared
            for cid in list(current_cells.keys()):
                if cid >= len(segments):
                    cells.append(current_cells.pop(cid))

            # Open new cells or extend existing
            for i, seg in enumerate(segments):
                if i not in current_cells:
                    current_cells[i] = {
                        'x_start': x,
                        'y_min': seg[0],
                        'y_max': seg[1],
                        'x_end': x
                    }
                else:
                    current_cells[i]['x_end'] = x
                    current_cells[i]['y_min'] = min(
                        current_cells[i]['y_min'], seg[0]
                    )
                    current_cells[i]['y_max'] = max(
                        current_cells[i]['y_max'], seg[1]
                    )

            prev_segments = segments
            x += slice_width

        # Close any remaining open cells
        for cid, cell in current_cells.items():
            cells.append(cell)

        # STEP 3 — generate lawnmower waypoints for each cell
        all_waypoints = []
        going_north = True

        for cell in cells:
            cell_waypoints = self.generate_lawnmower(
                cell['x_start'], cell['x_end'],
                cell['y_min'],   cell['y_max'],
                going_north
            )
            all_waypoints.extend(cell_waypoints)
            going_north = not going_north  # alternate direction cell-to-cell

        return all_waypoints

    def extract_y_segments(self, geom, miny, maxy):
        segments = []
        if geom.is_empty:
            return segments
        if geom.geom_type == 'Polygon':
            coords = list(geom.exterior.coords)
            ys = [c[1] for c in coords]
            segments.append((min(ys), max(ys)))
        elif geom.geom_type in ('MultiPolygon', 'GeometryCollection'):
            for g in geom.geoms:
                if g.geom_type == 'Polygon':
                    coords = list(g.exterior.coords)
                    ys = [c[1] for c in coords]
                    segments.append((min(ys), max(ys)))
        return segments

    # ------------------------------------------------------------------ #
    #  STEP 3 — lawnmower sweep within a cell                             #
    # ------------------------------------------------------------------ #
    def generate_lawnmower(self, x_start, x_end, y_min, y_max, going_north):
        waypoints = []
        x = x_start + self.sweep_spacing / 2

        while x <= x_end:
            if going_north:
                waypoints.append((x, y_min, self.sweep_altitude))
                waypoints.append((x, y_max, self.sweep_altitude))
            else:
                waypoints.append((x, y_max, self.sweep_altitude))
                waypoints.append((x, y_min, self.sweep_altitude))
            x += self.sweep_spacing
            going_north = not going_north

        return waypoints

    # ------------------------------------------------------------------ #
    #  ROS2 publishing                                                     #
    # ------------------------------------------------------------------ #
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = wp[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info('BCD path published.')

    def publish_next_waypoint(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints complete.')
            return

        wp = self.waypoints[self.current_waypoint_idx]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        pose.pose.position.z = wp[2]
        pose.pose.orientation.w = 1.0
        self.waypoint_pub.publish(pose)

    def advance_waypoint(self):
        self.current_waypoint_idx += 1
        self.get_logger().info(
            f'Advancing to waypoint {self.current_waypoint_idx}'
            f'/{len(self.waypoints)}'
        )

    def publish_markers(self):
        marker_array = MarkerArray()

        # Waypoint markers
        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp[0]
            m.pose.position.y = wp[1]
            m.pose.position.z = wp[2]
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = 0.2
            m.color.g = 0.6
            m.color.b = 1.0
            m.color.a = 0.8
            marker_array.markers.append(m)

        # Red zone markers
        for i, rz in enumerate(self.red_zones):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'red_zones'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            cx, cy = rz.centroid.x, rz.centroid.y
            bx = rz.bounds
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.05
            m.scale.x = bx[2] - bx[0]
            m.scale.y = bx[3] - bx[1]
            m.scale.z = 0.1
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 0.5
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = BCDPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
