#!/usr/bin/env python3
"""
qr_scanner_node.py
──────────────────
ROS2 node implementing the AeroTHON 2026 QR scanner pipeline.

Adapted from AeroTHON-main/QRCodeDetection/QRScannerNode.py to integrate
with the mission_manager service-based interface.

Works identically on:
  - Real hardware (RPi5 + RPi Camera Module 3 / OAK-D)
  - Gazebo simulation (camera plugin publishing sensor_msgs/Image)
  - SITL (uses sitl_qr_simulator publishing to /qr/detection directly)

The node operates in two modes, switched by the services below:
  - PHASE2: downward camera, single QR, store Delivery ID
  - PHASE5: downward camera, multi-QR, match against Delivery ID,
            project target to NED, publish detection for mission manager

Services (called by mission_manager/srv_helpers.py):
    /perception/scan_qr          (Trigger)  — Phase 2: scan reference QR, blocks until confirmed
    /perception/start_qr_scan    (Trigger)  — Phase 5: start async arena QR scanning

Subscriptions:
    /down_frame             (sensor_msgs/Image)     — camera frames
    /mavros/global_position/rel_alt     (std_msgs/Float64) — altitude above home
    /mavros/global_position/compass_hdg (std_msgs/Float64) — heading, deg CW from North
    /qr_scanner/delivery_id (std_msgs/String)        — inject stored ID (Phase 5)
    /mavros/global_position/global (sensor_msgs/NavSatFix) — GPS for detection publishing
    /camera_info            (sensor_msgs/CameraInfo)  — camera intrinsics

Publications:
    /qr_scanner/status      (std_msgs/String)        — human-readable status for debugging
    /qr/detection           (std_msgs/String)        — "content|qr_lat|qr_lon" for mission_node,
                                                       published ONLY after the confirmation gate

Parameters (set via ROS2 param file or command line):
    gate_required       int   (default 3)    — frames for confirmation gate
    min_confidence      float (default 0.5)  — minimum confidence to enter gate
    delivery_altitude   float (default 5.0)  — fallback altitude when rel_alt is unavailable
    scan_timeout_s      float (default 60.0) — Phase 2 service timeout
    cam_yaw_offset_deg  float (default 0.0)  — image-top direction vs nose, deg clockwise
    camera_fx/fy/cx/cy  float                — camera intrinsics from calibration

ROS2 dependencies:
    rclpy, sensor_msgs, std_msgs, std_srvs, cv_bridge
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg   import Image, CameraInfo, NavSatFix
from std_msgs.msg      import String, Float64
from std_srvs.srv      import Trigger

from cv_bridge import CvBridge

# ── Import pure pipeline logic ────────────────────────────────────────────────
from perception.qr_pipeline import (
    preprocess,
    detect_qr,
    store_delivery_id,
    find_matching_qr,
    project_to_ned,
    quad_centroid,
    ConfirmationGate,
    DETECTOR_NAME,
)


class QRScannerNode(Node):
    """
    ROS2 node for QR code detection in both Phase 2 and Phase 5.

    Mode switching:
        The node starts in IDLE mode and does nothing.
        /perception/scan_qr activates PHASE2, /perception/start_qr_scan PHASE5;
        each returns to IDLE once its gate confirms.
    """

    # Valid mode strings
    MODE_IDLE   = "IDLE"
    MODE_PHASE2 = "PHASE2"
    MODE_PHASE5 = "PHASE5"

    def __init__(self) -> None:
        super().__init__("qr_scanner_node")

        # ── Callback group for concurrent service handling ─────────────────
        self._cb_group = ReentrantCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("gate_required",     3)
        self.declare_parameter("min_confidence",    0.5)
        self.declare_parameter("delivery_altitude", 5.0)
        self.declare_parameter("scan_timeout_s",    60.0)
        self.declare_parameter("cam_yaw_offset_deg", 0.0)
        # Camera intrinsics — must be set from calibration or /camera_info
        # Defaults are approximate for RPi Camera Module 3 at 1280x720
        self.declare_parameter("camera_fx", 920.0)
        self.declare_parameter("camera_fy", 920.0)
        self.declare_parameter("camera_cx", 640.0)
        self.declare_parameter("camera_cy", 360.0)

        self._gate_required     = self.get_parameter("gate_required").value
        self._min_confidence    = self.get_parameter("min_confidence").value
        self._delivery_altitude = self.get_parameter("delivery_altitude").value
        self._scan_timeout_s    = float(self.get_parameter("scan_timeout_s").value)
        self._cam_yaw_offset    = float(self.get_parameter("cam_yaw_offset_deg").value)
        self._fx = self.get_parameter("camera_fx").value
        self._fy = self.get_parameter("camera_fy").value
        self._cx = self.get_parameter("camera_cx").value
        self._cy = self.get_parameter("camera_cy").value

        # ── State ──────────────────────────────────────────────────────────────
        self._mode          = self.MODE_IDLE
        self._gate          = ConfirmationGate(required=self._gate_required)
        self._delivery_id   : str | None = None   # set after Phase 2 confirms
        self._current_alt   : float | None = None   # rel_alt, metres above home
        self._heading_deg   : float | None = None   # compass heading
        self._bridge        = CvBridge()
        self._current_gps   = [None, None]  # [lat, lon] from MAVROS

        # Threading events for service blocking
        self._phase2_done_event = threading.Event()
        self._phase2_result     : str | None = None

        # ── QoS ───────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, "/down_frame", self._frame_cb, sensor_qos)

        self.create_subscription(
            String, "/qr_scanner/delivery_id", self._delivery_id_cb, latched_qos)

        self.create_subscription(
            Float64, "/mavros/global_position/rel_alt", self._rel_alt_cb, mavros_qos)
        self.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self._heading_cb, mavros_qos)

        self.create_subscription(
            CameraInfo, "/camera_info", self._camera_info_cb, reliable_qos)

        self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._gps_cb, mavros_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_status        = self.create_publisher(String,  "/qr_scanner/status",     reliable_qos)
        # Mission manager interface — "content|lat|lon" format
        self._pub_qr_detection  = self.create_publisher(String,  "/qr/detection",          reliable_qos)

        # ── Services (called by mission_manager/srv_helpers.py) ───────────────
        self.create_service(
            Trigger, "/perception/scan_qr",
            self._handle_scan_qr,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/perception/start_qr_scan",
            self._handle_start_qr_scan,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"[qr_scanner] Started | detector: {DETECTOR_NAME} | "
            f"gate={self._gate_required} | min_conf={self._min_confidence}"
        )
        self.get_logger().info(
            "[qr_scanner] Services: /perception/scan_qr, /perception/start_qr_scan"
        )
        self.get_logger().info("[qr_scanner] Waiting for a service call...")

    # ═════════════════════════════════════════════════════════════════════════
    # Service handlers (called by mission_manager/srv_helpers.py)
    # ═════════════════════════════════════════════════════════════════════════

    def _handle_scan_qr(self, request, response):
        """
        Phase 2: Scan reference QR at takeoff zone.
        Blocks until the confirmation gate fires, then returns the decoded ID.

        Called by: mission_node._scan_reference_qr()
        """
        self.get_logger().info("[qr_scanner] /perception/scan_qr called — activating PHASE2")

        # Reset state and activate PHASE2
        self._delivery_id = None
        self._gate.reset()
        self._phase2_done_event.clear()
        self._phase2_result = None
        self._mode = self.MODE_PHASE2
        self._publish_status("PHASE2_ACTIVE")

        # Block until Phase2 confirms or timeout. mission_node waits longer
        # than this (qr_scan_timeout_s + margin), so it always gets an answer.
        confirmed = self._phase2_done_event.wait(timeout=self._scan_timeout_s)

        if confirmed and self._phase2_result:
            response.success = True
            response.message = self._phase2_result
            self.get_logger().info(
                f"[qr_scanner] Phase 2 confirmed: '{self._phase2_result}'"
            )
        else:
            response.success = False
            response.message = "QR scan timed out or failed"
            self.get_logger().warn("[qr_scanner] Phase 2 scan timed out")
            self._mode = self.MODE_IDLE

        return response

    def _handle_start_qr_scan(self, request, response):
        """
        Phase 5: Start async QR scanning during arena search.
        Returns immediately — detections are published to /qr/detection.

        Called by: mission_node._arena_search()
        """
        self.get_logger().info(
            "[qr_scanner] /perception/start_qr_scan called — activating PHASE5"
        )

        # Reset gate for Phase 5
        self._gate.reset()
        self._mode = self.MODE_PHASE5
        self._publish_status("PHASE5_ACTIVE")

        response.success = True
        response.message = "QR scanning started (Phase 5)"
        return response

    # ═════════════════════════════════════════════════════════════════════════
    # Subscriber callbacks
    # ═════════════════════════════════════════════════════════════════════════

    def _delivery_id_cb(self, msg: String) -> None:
        """
        The mission FSM publishes the target ID it locked before Phase 5.
        The FSM is the authority: its ID replaces whatever Phase 2 stored
        (they are the same in normal operation).
        """
        new_id = msg.data.strip()
        if new_id and new_id != self._delivery_id:
            self._delivery_id = new_id
            self.get_logger().info(
                f"[qr_scanner] Delivery ID set for Phase 5: '{self._delivery_id}'"
            )

    def _rel_alt_cb(self, msg: Float64) -> None:
        self._current_alt = float(msg.data)

    def _heading_cb(self, msg: Float64) -> None:
        self._heading_deg = float(msg.data)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        """Update camera intrinsics from /camera_info if available."""
        if len(msg.k) == 9:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]

    def _gps_cb(self, msg: NavSatFix) -> None:
        """Track current GPS position for /qr/detection publishing."""
        self._current_gps[0] = msg.latitude
        self._current_gps[1] = msg.longitude

    def _frame_cb(self, msg: Image) -> None:
        """
        Main pipeline callback — called on every incoming camera frame.
        Dispatches to Phase 2 or Phase 5 handler based on current mode.
        """
        if self._mode == self.MODE_IDLE:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"[qr_scanner] CvBridge conversion failed: {e}")
            return

        # ── Pre-processing (same for both phases) ─────────────────────────────
        processed  = preprocess(frame)
        detections = detect_qr(processed, min_confidence=self._min_confidence)

        # ── Dispatch to phase handler ─────────────────────────────────────────
        if self._mode == self.MODE_PHASE2:
            self._handle_phase2(detections)
        elif self._mode == self.MODE_PHASE5:
            self._handle_phase5(detections)

    # ═════════════════════════════════════════════════════════════════════════
    # Phase 2 — Initial QR scan
    # ═════════════════════════════════════════════════════════════════════════

    def _handle_phase2(self, detections: list[dict]) -> None:
        """
        Phase 2: drone is hovering at 5 m above start zone QR.
        Feed best detection into gate. On confirmation, store the Delivery
        ID, switch to IDLE and wake the blocking /perception/scan_qr call.
        """
        if self._delivery_id is not None:
            # Already confirmed — nothing more to do
            return

        best = detections[0]["data"] if detections else None

        if self._gate.update(best):
            # Gate confirmed
            self._delivery_id = store_delivery_id(self._gate.confirmed)
            self.get_logger().info(
                f"[qr_scanner] [Phase 2] DELIVERY ID CONFIRMED: '{self._delivery_id}'"
            )

            self._publish_status(f"PHASE2_CONFIRMED:{self._delivery_id}")
            self._mode = self.MODE_IDLE   # FSM takes over

            # Signal the blocking service call
            self._phase2_result = self._delivery_id
            self._phase2_done_event.set()

        else:
            self._publish_status(
                f"PHASE2_SCANNING gate={self._gate.progress} "
                f"best={best or 'none'}"
            )

    # ═════════════════════════════════════════════════════════════════════════
    # Phase 5 — Delivery zone multi-QR scan
    # ═════════════════════════════════════════════════════════════════════════

    def _handle_phase5(self, detections: list[dict]) -> None:
        """
        Phase 5: drone navigating delivery zone at altitude.
        When a QR is detected, publish to /qr/detection in the format
        expected by mission_node._qr_detection_cb: "content|lat|lon"
        """
        if self._delivery_id is None:
            self.get_logger().warn(
                "[qr_scanner] [Phase 5] No Delivery ID set — cannot match. "
                "Publish ID to /qr_scanner/delivery_id first.",
                throttle_duration_sec=5.0,
            )
            return

        # ── Match against Delivery ID ─────────────────────────────────────────
        matching = find_matching_qr(detections, self._delivery_id)
        gate_value = matching["data"] if matching else None

        if self._gate.update(gate_value):
            # Gate confirmed on matching QR
            self.get_logger().info(
                f"[qr_scanner] [Phase 5] TARGET CONFIRMED: '{self._gate.confirmed}'"
            )

            # ── Project the QR to a ground position ───────────────────────────
            pts = matching["points"]
            u, v = quad_centroid(pts)
            altitude = self._current_alt if (self._current_alt or 0.0) > 1.0 \
                else self._delivery_altitude
            heading = self._heading_deg if self._heading_deg is not None else 0.0
            if self._heading_deg is None:
                self.get_logger().warn("[qr_scanner] [Phase 5] No heading — assuming North")

            d_north, d_east = project_to_ned(
                pixel_u=u, pixel_v=v,
                altitude_m=altitude,
                cx=self._cx, cy=self._cy,
                fx=self._fx, fy=self._fy,
                heading_deg=heading,
                cam_yaw_offset_deg=self._cam_yaw_offset,
            )

            # ── Publish "content|qr_lat|qr_lon" for mission_node ──────────────
            if self._current_gps[0] is not None:
                lat0, lon0 = self._current_gps
                qr_lat = lat0 + d_north / 111_320.0
                qr_lon = lon0 + d_east / (111_320.0 * math.cos(math.radians(lat0)))
                det_msg = String()
                det_msg.data = f"{self._gate.confirmed}|{qr_lat:.7f}|{qr_lon:.7f}"
                self._pub_qr_detection.publish(det_msg)
                self.get_logger().info(
                    f"[qr_scanner] [Phase 5] Published /qr/detection: {det_msg.data}"
                )
            else:
                self.get_logger().warn(
                    "[qr_scanner] [Phase 5] No GPS fix — cannot publish detection with coords"
                )

            self.get_logger().info(
                f"[qr_scanner] [Phase 5] offset N={d_north:.2f} m, E={d_east:.2f} m "
                f"(alt={altitude:.1f} m, hdg={heading:.0f}, pixel=({u:.0f},{v:.0f}))"
            )

            self._publish_status(
                f"PHASE5_CONFIRMED d_north={d_north:.2f} d_east={d_east:.2f}"
            )
            self._mode = self.MODE_IDLE   # FSM takes over for the drop

        else:
            # Unconfirmed detections are NOT published to /qr/detection: a
            # single-frame misread must never trigger the drop.
            status_parts = [f"PHASE5_SCANNING gate={self._gate.progress}"]
            if detections:
                status_parts.append(
                    f"best_conf={max(d['confidence'] for d in detections):.2f}"
                )
            status_parts.append(f"match={'yes' if matching else 'no'}")
            self._publish_status(" ".join(status_parts))

    # ═════════════════════════════════════════════════════════════════════════
    # Helpers
    # ═════════════════════════════════════════════════════════════════════════

    def _publish_status(self, text: str) -> None:
        msg      = String()
        msg.data = text
        self._pub_status.publish(msg)


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None) -> None:
    rclpy.init(args=args)
    node = QRScannerNode()
    try:
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
