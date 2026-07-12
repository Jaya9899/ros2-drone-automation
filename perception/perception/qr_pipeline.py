"""
qr_pipeline.py
──────────────
Pure pipeline logic for QR code detection.
No ROS2. No camera hardware. No display.
Fully testable in isolation.

Import this module in:
  - qr_scanner_node.py  (ROS2 node — real hardware + Gazebo)
  - test_qr_pipeline.py (unit tests)
  - qr_pipeline_demo_v3.py (standalone demo)
"""

from __future__ import annotations
from collections import deque

import cv2
import numpy as np


# ── Detector (module-level singleton) ────────────────────────────────────────
try:
    _detector = cv2.QRCodeDetectorAruco()
    DETECTOR_NAME = "QRCodeDetectorAruco"
except AttributeError:
    _detector = cv2.QRCodeDetector()
    DETECTOR_NAME = "QRCodeDetector"


# ═════════════════════════════════════════════════════════════════════════════
# Stage 1 — Pre-processing
# ═════════════════════════════════════════════════════════════════════════════

def preprocess(frame: np.ndarray) -> np.ndarray:
    """
    Convert a BGR frame to a noise-reduced, contrast-enhanced greyscale image.

    Pipeline:
      1. Greyscale — removes colour (QR codes carry no colour information)
      2. CLAHE     — local contrast enhancement for uneven outdoor lighting
      3. Gaussian blur (5x5) — reduces CMOS sensor noise before detection

    Args:
        frame: BGR uint8 numpy array from camera or ROS Image bridge.

    Returns:
        Single-channel uint8 greyscale image, same spatial dimensions as input.
    """
    grey    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe   = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    eq      = clahe.apply(grey)
    blurred = cv2.GaussianBlur(eq, (5, 5), 0)
    return blurred


# ═════════════════════════════════════════════════════════════════════════════
# Stage 2 — Confidence scoring
# ═════════════════════════════════════════════════════════════════════════════

def quad_confidence(pts: np.ndarray | None) -> float:
    """
    Geometric confidence score [0.0, 1.0] for a detected QR bounding quad.

    Three equal-weight sub-scores averaged:
      1. Rectangularity  — how close each corner angle is to 90 degrees
      2. Area            — normalised quad area (min 400 px, sat 4000 px)
      3. Aspect ratio    — shorter/longer opposite-side pair (1.0 = head-on)

    Used to:
      - Gate entry into the confirmation buffer (threshold 0.5)
      - Trigger a forward movement stop (threshold 0.7, Phase 5 only)

    Args:
        pts: (N, 2) int array of corner pixel coordinates, or None.

    Returns:
        float in [0.0, 1.0]. Returns 0.0 if pts is None or has fewer than 4 points.
    """
    if pts is None or len(pts) < 4:
        return 0.0

    pts = pts.reshape(4, 2).astype(np.float32)

    # 1. Rectangularity
    angles = []
    for i in range(4):
        v1 = pts[(i - 1) % 4] - pts[i]
        v2 = pts[(i + 1) % 4] - pts[i]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            angles.append(0.0)
            continue
        cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        dev = abs(np.degrees(np.arccos(cos_a)) - 90.0)
        angles.append(max(0.0, 1.0 - dev / 45.0))
    rect_score = float(np.mean(angles))

    # 2. Area (Shoelace formula)
    x, y = pts[:, 0], pts[:, 1]
    area = 0.5 * abs(
        x[0] * y[1] - x[1] * y[0] +
        x[1] * y[2] - x[2] * y[1] +
        x[2] * y[3] - x[3] * y[2] +
        x[3] * y[0] - x[0] * y[3]
    )
    area_score = float(np.clip((area - 400.0) / (4000.0 - 400.0), 0.0, 1.0))

    # 3. Aspect ratio
    sides = [np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)]
    shorter = min(sides[0] + sides[2], sides[1] + sides[3])
    longer  = max(sides[0] + sides[2], sides[1] + sides[3])
    aspect_score = float(shorter / longer) if longer > 1e-6 else 0.0

    return round((rect_score + area_score + aspect_score) / 3.0, 3)


# ═════════════════════════════════════════════════════════════════════════════
# Stage 3 — QR detection
# ═════════════════════════════════════════════════════════════════════════════

def detect_qr(
    processed: np.ndarray,
    min_confidence: float = 0.5,
) -> list[dict]:
    """
    Detect and decode all QR codes in a pre-processed frame.

    Uses cv2.QRCodeDetectorAruco (opencv-contrib 4.8+) or falls back to
    cv2.QRCodeDetector on older builds. detectAndDecodeMulti handles
    both Phase 2 (single code) and Phase 5 (multiple codes) transparently.

    Only detections with quad_confidence above min_confidence are returned.

    Args:
        processed:      Single-channel greyscale image from preprocess().
        min_confidence: Minimum quad confidence to include in results (default 0.5).

    Returns:
        List of dicts: { 'data': str, 'points': ndarray(4,2), 'confidence': float }
        Empty list if nothing detected or all detections below threshold.
    """
    results = []

    try:
        ret, decoded_list, points_list, _ = _detector.detectAndDecodeMulti(processed)
        if ret and decoded_list:
            for data, pts in zip(decoded_list, points_list):
                if not data:
                    continue
                pts_arr    = pts.reshape(-1, 2).astype(int)
                confidence = quad_confidence(pts_arr)
                if confidence >= min_confidence:
                    results.append({
                        "data":       data.strip(),
                        "points":     pts_arr,
                        "confidence": confidence,
                    })

    except (cv2.error, AttributeError):
        # Older OpenCV — single QR fallback
        try:
            data, pts, _ = _detector.detectAndDecode(processed)
            if data:
                pts_arr    = pts.reshape(-1, 2).astype(int) if pts is not None else None
                confidence = quad_confidence(pts_arr)
                if confidence >= min_confidence:
                    results.append({
                        "data":       data.strip(),
                        "points":     pts_arr,
                        "confidence": confidence,
                    })
        except cv2.error:
            pass

    return results


# ═════════════════════════════════════════════════════════════════════════════
# Stage 4 — Delivery ID storage
# ═════════════════════════════════════════════════════════════════════════════

def store_delivery_id(decoded_string: str) -> str:
    """
    Store the confirmed decoded string verbatim as the Delivery ID.

    No lookup table, no encoding interpretation, no format assumption.
    Whatever the start zone QR contains becomes the Delivery ID.
    In Phase 5, the delivery zone QR whose content exactly matches this
    string is the payload target.

    Args:
        decoded_string: Confirmed decoded QR string from the gate.

    Returns:
        Stripped string — the Delivery ID.
    """
    return decoded_string.strip()


# ═════════════════════════════════════════════════════════════════════════════
# Stage 5 — Confirmation gate
# ═════════════════════════════════════════════════════════════════════════════

class ConfirmationGate:
    """
    3-frame confirmation gate.

    Accepts a detection only when the same decoded string appears in
    `required` consecutive frames without interruption. Any mismatch
    (including a frame with no detection) resets the buffer.

    Returns True exactly once per gate lifetime — a `fired` flag prevents
    re-triggering on subsequent identical frames. Call reset() to reuse
    the gate for a new scan phase.

    Usage:
        gate = ConfirmationGate(required=3)
        for frame in camera:
            detections = detect_qr(preprocess(frame))
            best = detections[0]['data'] if detections else None
            if gate.update(best):
                delivery_id = store_delivery_id(gate.confirmed)
    """

    def __init__(self, required: int = 3) -> None:
        self.required  = required
        self._buffer   = deque(maxlen=required)
        self.confirmed: str | None = None
        self._fired    = False

    def update(self, value: str | None) -> bool:
        """
        Feed one frame result. Returns True on the first confirming frame only.

        Args:
            value: Decoded string from this frame, or None if no detection.

        Returns:
            True exactly once when gate confirms. False at all other times.
        """
        if value is None:
            self._buffer.clear()
            return False

        self._buffer.append(value)

        if self._fired:
            return False

        if (len(self._buffer) == self.required
                and len(set(self._buffer)) == 1):
            self.confirmed = value
            self._fired    = True
            return True

        return False

    def reset(self) -> None:
        """Clear buffer and allow gate to fire again."""
        self._buffer.clear()
        self.confirmed = None
        self._fired    = False

    @property
    def progress(self) -> str:
        """Human-readable gate fill status, e.g. '2/3'."""
        if not self._buffer:
            return f"0/{self.required}"
        latest = self._buffer[-1]
        count  = sum(1 for x in self._buffer if x == latest)
        return f"{count}/{self.required}"


# ═════════════════════════════════════════════════════════════════════════════
# Phase 5 — Delivery zone matching
# ═════════════════════════════════════════════════════════════════════════════

def find_matching_qr(
    detections: list[dict],
    delivery_id: str,
) -> dict | None:
    """
    Find the delivery zone QR code whose decoded content exactly matches
    the stored Delivery ID.

    Exact string match only — no partial matching, no fuzzy comparison.
    Case-sensitive. Both strings are stripped before comparison.

    Args:
        detections:  List of detection dicts from detect_qr().
        delivery_id: The Delivery ID string confirmed in Phase 2.

    Returns:
        The matching detection dict, or None if no match found.
    """
    for det in detections:
        if det["data"].strip() == delivery_id.strip():
            return det
    return None


def project_to_ned(
    pixel_u: float,
    pixel_v: float,
    altitude_m: float,
    cx: float,
    cy: float,
    fx: float,
    fy: float,
) -> tuple[float, float]:
    """
    Project a pixel coordinate to a NED ground-plane offset using the
    pinhole camera model.

    dx = (u - cx) * Z / fx
    dy = (v - cy) * Z / fy

    Args:
        pixel_u:    Pixel x-coordinate of target centroid.
        pixel_v:    Pixel y-coordinate of target centroid.
        altitude_m: Known hover altitude in metres (Z).
        cx, cy:     Principal point from camera calibration.
        fx, fy:     Focal lengths in pixels from camera calibration.

    Returns:
        (delta_north, delta_east) NED offset in metres.
        Add to current LOCAL_POSITION_NED to get absolute target waypoint.
    """
    delta_x = (pixel_u - cx) * altitude_m / fx   # North offset
    delta_y = (pixel_v - cy) * altitude_m / fy   # East offset
    return float(delta_x), float(delta_y)


def quad_centroid(pts: np.ndarray) -> tuple[float, float]:
    """
    Compute the centroid (u, v) of a 4-point bounding quadrilateral.

    Args:
        pts: (4, 2) array of corner pixel coordinates.

    Returns:
        (u, v) pixel centroid.
    """
    pts = pts.reshape(4, 2).astype(np.float32)
    return float(pts[:, 0].mean()), float(pts[:, 1].mean())