#!/usr/bin/env python3
"""
depth_processor_node.py
───────────────────────
ROS2 node for processing OAK-D depth data for obstacle detection.

Adapted from AeroTHON-main/rgb_depth_fusion.py — converted from a
standalone DepthAI script to a ROS2 node subscribing to OAK-D topics
published by the depthai-ros driver.

This node provides obstacle distance information that feeds the
RL-based obstacle avoidance system (when integrated).

Subscriptions:
    /oak/stereo/image_raw   (sensor_msgs/Image)     — depth image (uint16, mm)
    /oak/rgb/image_raw      (sensor_msgs/Image)     — RGB image

Publications:
    /obstacle/min_distance  (std_msgs/Float32)       — closest obstacle in metres
    /obstacle/depth_image   (sensor_msgs/Image)      — colorized depth for debugging

Parameters:
    roi_width           int   (default 100)   — width of centre ROI for obstacle check
    roi_height          int   (default 100)   — height of centre ROI
    min_valid_depth_mm  int   (default 300)   — ignore depth values below this (noise)
    warning_distance_m  float (default 1.5)   — distance to log a warning
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge

import numpy as np
import cv2


class DepthProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_processor_node")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("roi_width", 100)
        self.declare_parameter("roi_height", 100)
        self.declare_parameter("min_valid_depth_mm", 300)
        self.declare_parameter("warning_distance_m", 1.5)

        self._roi_w = self.get_parameter("roi_width").value
        self._roi_h = self.get_parameter("roi_height").value
        self._min_valid_mm = self.get_parameter("min_valid_depth_mm").value
        self._warning_m = self.get_parameter("warning_distance_m").value

        self._bridge = CvBridge()

        # ── QoS ───────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(
            Image, "/oak/stereo/image_raw",
            self._depth_cb, sensor_qos)

        # ── Publishers ────────────────────────────────────────────────
        self._pub_min_dist = self.create_publisher(
            Float32, "/obstacle/min_distance", 10)
        self._pub_depth_vis = self.create_publisher(
            Image, "/obstacle/depth_image", sensor_qos)

        self.get_logger().info(
            f"[depth_processor] Ready — ROI {self._roi_w}x{self._roi_h}, "
            f"warning at {self._warning_m}m"
        )

    def _depth_cb(self, msg: Image) -> None:
        """Process incoming depth frame from OAK-D stereo pair."""
        try:
            # OAK-D publishes depth as 16UC1 (uint16, values in mm)
            depth_frame = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"[depth_processor] CvBridge failed: {e}")
            return

        h, w = depth_frame.shape[:2]
        cx, cy = w // 2, h // 2

        # ── Extract centre ROI ────────────────────────────────────────
        half_w = self._roi_w // 2
        half_h = self._roi_h // 2
        y1 = max(0, cy - half_h)
        y2 = min(h, cy + half_h)
        x1 = max(0, cx - half_w)
        x2 = min(w, cx + half_w)
        roi = depth_frame[y1:y2, x1:x2]

        # ── Find minimum valid depth ─────────────────────────────────
        valid_mask = roi > self._min_valid_mm
        if np.any(valid_mask):
            min_dist_mm = int(np.min(roi[valid_mask]))
            min_dist_m = min_dist_mm / 1000.0
        else:
            min_dist_m = float('inf')

        # ── Publish min distance ──────────────────────────────────────
        dist_msg = Float32()
        dist_msg.data = min_dist_m
        self._pub_min_dist.publish(dist_msg)

        if min_dist_m < self._warning_m:
            self.get_logger().warn(
                f"[depth_processor] OBSTACLE at {min_dist_m:.2f}m "
                f"(warning threshold: {self._warning_m}m)"
            )

        # ── Publish colorized depth image for debugging ───────────────
        try:
            depth_vis = cv2.normalize(
                depth_frame, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = cv2.applyColorMap(
                depth_vis.astype(np.uint8), cv2.COLORMAP_JET)

            # Draw ROI rectangle
            cv2.rectangle(depth_vis, (x1, y1), (x2, y2), (255, 255, 255), 2)

            # Add distance text
            cv2.putText(
                depth_vis,
                f"Min: {min_dist_m:.2f}m",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
            )

            vis_msg = self._bridge.cv2_to_imgmsg(depth_vis, encoding="bgr8")
            vis_msg.header = msg.header
            self._pub_depth_vis.publish(vis_msg)
        except Exception:
            pass  # Visualization is non-critical


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
