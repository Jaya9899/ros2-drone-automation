# srv_helpers.py — ROS2 service-call wrappers for path_planner and perception
#
# Uses std_srvs/Trigger as a stopgap until proper custom .srv files are
# defined in the path_planner and perception packages.
#
# Usage:
#   from mission_manager.srv_helpers import call_fly_corridor, call_run_lawnmower, call_start_qr_scan
#   call_fly_corridor(node, speed=2.0, obstacles=False)
#   call_run_lawnmower(node, speed=1.5)
#   call_start_qr_scan(node, target_qr_id="QR_TARGET_DEMO_ID")

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def _call_trigger(node: Node, srv_name: str, timeout: float = 10.0):
    """Call a Trigger-type service and return the result.

    Uses poll-based wait instead of spin_until_future_complete so it is
    safe to call from a background thread while the main thread spins.
    """
    import time

    client = node.create_client(Trigger, srv_name)

    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().warn(
            f"[srv_helpers] Service {srv_name} not available after {timeout}s — "
            f"continuing without it (service may not be running yet)"
        )
        return None

    future = client.call_async(Trigger.Request())

    # Poll-based wait — safe when main thread already spins the node
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)

    if future.result() is None:
        node.get_logger().error(f"[srv_helpers] Call to {srv_name} returned no result")
        return None

    result = future.result()
    node.get_logger().info(
        f"[srv_helpers] {srv_name} -> success={result.success}, "
        f"message='{result.message}'"
    )
    return result


# ---------------------------------------------------------------------------
# Path Planner helpers
# ---------------------------------------------------------------------------

def call_fly_corridor(node: Node, speed: float, obstacles: bool,
                      timeout: float = 10.0) -> bool:
    """
    Ask the path_planner node to fly through the corridor.

    Parameters
    ----------
    node : ROS2 node handle
    speed : cruise speed in m/s
    obstacles : True if the corridor has static obstacles (corridor 2)
    timeout : service call timeout in seconds

    Note: speed and obstacles are logged but NOT sent to the service because
    std_srvs/Trigger has no request fields.  Replace with a custom .srv
    when available.
    """
    tag = "with obstacles" if obstacles else "clean"
    node.get_logger().info(
        f"[srv_helpers] Requesting fly_corridor ({tag}, {speed} m/s)"
    )

    result = _call_trigger(node, "/path_planner/fly_corridor", timeout=timeout)

    if result is None:
        node.get_logger().warn(
            "[srv_helpers] fly_corridor service unavailable — skipping"
        )
        return False

    return result.success


def call_run_lawnmower(node: Node, speed: float,
                       timeout: float = 10.0) -> bool:
    """
    Ask the path_planner node to execute the BCD lawnmower search pattern.

    Parameters
    ----------
    node : ROS2 node handle
    speed : arena cruise speed in m/s
    timeout : service call timeout in seconds
    """
    node.get_logger().info(
        f"[srv_helpers] Requesting run_lawnmower ({speed} m/s)"
    )

    result = _call_trigger(node, "/path_planner/run_lawnmower", timeout=timeout)

    if result is None:
        node.get_logger().warn(
            "[srv_helpers] run_lawnmower service unavailable — skipping"
        )
        return False

    return result.success


def call_start_qr_scan(node: Node, target_qr_id: str,
                        timeout: float = 10.0) -> bool:
    """
    Ask the perception node to begin scanning for the target QR code.

    Parameters
    ----------
    node : ROS2 node handle
    target_qr_id : the QR code string to match against
    timeout : service call timeout in seconds
    """
    node.get_logger().info(
        f"[srv_helpers] Requesting start_qr_scan (target='{target_qr_id}')"
    )

    result = _call_trigger(node, "/perception/start_qr_scan", timeout=timeout)

    if result is None:
        node.get_logger().warn(
            "[srv_helpers] start_qr_scan service unavailable — skipping"
        )
        return False

    return result.success


def call_scan_reference_qr(node: Node, timeout: float = 10.0) -> str | None:
    """
    Ask the perception node to decode the reference QR at the takeoff zone.

    Returns the decoded QR string, or None on failure.
    """
    node.get_logger().info(
        "[srv_helpers] Requesting scan_reference_qr"
    )

    result = _call_trigger(node, "/perception/scan_qr", timeout=timeout)

    if result is None:
        node.get_logger().warn(
            "[srv_helpers] scan_qr service unavailable — using fallback"
        )
        return None

    if result.success:
        return result.message  # decoded QR string
    return None


# ---------------------------------------------------------------------------
# Payload helpers
# ---------------------------------------------------------------------------

def call_payload_release(node: Node, timeout: float = 10.0) -> bool:
    """
    Ask the payload node to release/detach the payload from the mechanism.
    """
    node.get_logger().info("[srv_helpers] Requesting payload/release")

    result = _call_trigger(node, "/payload/release", timeout=timeout)

    if result is None:
        node.get_logger().warn(
            "[srv_helpers] payload/release service unavailable — skipping"
        )
        return False

    return result.success


# ---------------------------------------------------------------------------
# Lawnmower stop
# ---------------------------------------------------------------------------

def call_stop_lawnmower(node: Node, timeout: float = 10.0) -> bool:
    """
    Ask the path_planner node to stop the current lawnmower search pattern.
    Called when the target QR has been matched and the drone should
    break out of the search.
    """
    node.get_logger().info("[srv_helpers] Requesting stop_lawnmower")

    result = _call_trigger(node, "/path_planner/stop_lawnmower", timeout=timeout)

    if result is None:
        node.get_logger().warn(
            "[srv_helpers] stop_lawnmower service unavailable — skipping"
        )
        return False

    return result.success
