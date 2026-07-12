# mavros_utils.py — Synchronous wrappers for common MAVROS operations
#
# Usage:
#   from mission_manager.mavros_utils import arm, set_mode, takeoff, wait_for_altitude
#   set_mode(node, "GUIDED")
#   arm(node)
#   takeoff(node, altitude_m=5.0)
#   wait_for_altitude(node, target_alt=5.0)
#
# All functions accept a ROS2 node handle as the first argument so they can
# create service clients and subscriptions without owning their own node.
#
# All timeout / tolerance values are passed as keyword arguments — no
# module-level constants.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

# QoS profile for high-frequency telemetry / sensors
# MAVROS often publishes these as BEST_EFFORT
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


def _call_service(node: Node, srv_type, srv_name: str, request,
                  timeout: float = 10.0):
    """Create a client, wait for the service, call it synchronously, return result.

    Uses poll-based wait instead of spin_until_future_complete so it is
    safe to call from a background thread while the main thread spins.
    """
    import time

    client = node.create_client(srv_type, srv_name)

    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().error(f"Service {srv_name} not available after {timeout}s")
        return None

    future = client.call_async(request)

    # Poll-based wait — safe when main thread already spins the node
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)

    if future.result() is None:
        node.get_logger().error(f"Service call to {srv_name} failed (no result)")
        return None

    return future.result()


# ---------------------------------------------------------------------------
# set_mode  — e.g. "GUIDED", "STABILIZE", "LAND"
# ---------------------------------------------------------------------------
def set_mode(node: Node, mode: str, timeout: float = 10.0) -> bool:
    """Switch the FCU flight mode via /mavros/set_mode."""
    req = SetMode.Request()
    req.custom_mode = mode

    result = _call_service(node, SetMode, "/mavros/set_mode", req, timeout=timeout)

    if result and result.mode_sent:
        node.get_logger().info(f"[mavros_utils] Mode set to {mode}")
        return True

    node.get_logger().error(f"[mavros_utils] Failed to set mode {mode}")
    return False


# ---------------------------------------------------------------------------
# arm / disarm
# ---------------------------------------------------------------------------
def arm(node: Node, timeout: float = 10.0) -> bool:
    """Arm the vehicle via /mavros/cmd/arming."""
    req = CommandBool.Request()
    req.value = True

    result = _call_service(node, CommandBool, "/mavros/cmd/arming", req, timeout=timeout)

    if result and result.success:
        node.get_logger().info("[mavros_utils] Vehicle armed")
        return True

    node.get_logger().error("[mavros_utils] Arming failed")
    return False


# ---------------------------------------------------------------------------
# takeoff
# ---------------------------------------------------------------------------
def takeoff(node: Node, altitude_m: float, timeout: float = 10.0) -> bool:
    """Command takeoff to *altitude_m* via /mavros/cmd/takeoff."""
    req = CommandTOL.Request()
    req.altitude = float(altitude_m)
    req.latitude = 0.0   # 0 = use current position
    req.longitude = 0.0
    req.min_pitch = 0.0
    req.yaw = 0.0

    result = _call_service(node, CommandTOL, "/mavros/cmd/takeoff", req, timeout=timeout)

    if result and result.success:
        node.get_logger().info(f"[mavros_utils] Takeoff command accepted ({altitude_m} m)")
        return True

    node.get_logger().error("[mavros_utils] Takeoff command rejected")
    return False


# ---------------------------------------------------------------------------
# wait_for_altitude — blocks until drone reaches target altitude
# ---------------------------------------------------------------------------
def wait_for_altitude(
    node: Node,
    target_alt: float,
    tolerance: float = 0.5,
    timeout: float = 30.0,
) -> bool:
    """Subscribe to local position and spin until altitude is within tolerance."""
    import time

    reached = False
    current_alt = 0.0

    def _pose_cb(msg: PoseStamped):
        nonlocal current_alt, reached
        current_alt = msg.pose.position.z
        if abs(current_alt - target_alt) <= tolerance:
            reached = True

    sub = node.create_subscription(
        PoseStamped,
        "/mavros/local_position/pose",
        _pose_cb,
        sensor_qos,
    )

    deadline = time.time() + timeout
    while not reached and time.time() < deadline:
        time.sleep(0.25)

    # cleanup
    node.destroy_subscription(sub)

    if reached:
        node.get_logger().info(
            f"[mavros_utils] Altitude reached: {current_alt:.1f} m "
            f"(target {target_alt:.1f} m)"
        )
    else:
        node.get_logger().warn(
            f"[mavros_utils] Altitude timeout: {current_alt:.1f} m "
            f"after {timeout}s (target {target_alt:.1f} m)"
        )

    return reached


# ---------------------------------------------------------------------------
# disarm
# ---------------------------------------------------------------------------
def disarm(node: Node, timeout: float = 10.0) -> bool:
    """Disarm the vehicle via /mavros/cmd/arming."""
    req = CommandBool.Request()
    req.value = False

    result = _call_service(node, CommandBool, "/mavros/cmd/arming", req, timeout=timeout)

    if result and result.success:
        node.get_logger().info("[mavros_utils] Vehicle disarmed")
        return True

    node.get_logger().error("[mavros_utils] Disarm failed")
    return False


# ---------------------------------------------------------------------------
# land — switch to LAND mode and wait for touchdown
# ---------------------------------------------------------------------------
def land(node: Node, timeout: float = 10.0) -> bool:
    """Command landing by switching to LAND mode via /mavros/cmd/land."""
    req = CommandTOL.Request()
    req.altitude = 0.0
    req.latitude = 0.0   # 0 = use current position
    req.longitude = 0.0
    req.min_pitch = 0.0
    req.yaw = 0.0

    result = _call_service(node, CommandTOL, "/mavros/cmd/land", req, timeout=timeout)

    if result and result.success:
        node.get_logger().info("[mavros_utils] Land command accepted")
        return True

    # Fallback: try switching flight mode to LAND
    node.get_logger().warn(
        "[mavros_utils] /mavros/cmd/land failed, falling back to set_mode LAND"
    )
    return set_mode(node, "LAND", timeout=timeout)


# ---------------------------------------------------------------------------
# goto — fly to a GPS position at a given altitude
# ---------------------------------------------------------------------------
def goto(
    node: Node,
    lat: float,
    lon: float,
    altitude_m: float,
    publish_count: int = 5,
) -> bool:
    """
    Publish a setpoint to /mavros/setpoint_position/global to navigate
    the vehicle to (lat, lon, altitude_m).

    Uses mavros_msgs/GlobalPositionTarget via the SET_POSITION_TARGET_GLOBAL_INT
    interface.  Requires GUIDED mode.
    """
    from mavros_msgs.msg import GlobalPositionTarget
    from std_msgs.msg import Header

    import time as _time

    pub = node.create_publisher(
        GlobalPositionTarget,
        "/mavros/setpoint_position/global",
        10,
    )

    msg = GlobalPositionTarget()
    msg.header = Header()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
    msg.type_mask = (
        GlobalPositionTarget.IGNORE_VX |
        GlobalPositionTarget.IGNORE_VY |
        GlobalPositionTarget.IGNORE_VZ |
        GlobalPositionTarget.IGNORE_AFX |
        GlobalPositionTarget.IGNORE_AFY |
        GlobalPositionTarget.IGNORE_AFZ |
        GlobalPositionTarget.IGNORE_YAW_RATE
    )
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = altitude_m

    # Publish a few times to make sure the FCU accepts it
    # Use time.sleep instead of spin_once — safe from background threads
    for _ in range(publish_count):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        _time.sleep(0.1)

    node.get_logger().info(
        f"[mavros_utils] Goto command sent: lat={lat:.7f}, lon={lon:.7f}, "
        f"alt={altitude_m:.1f} m"
    )

    # Clean up the publisher
    node.destroy_publisher(pub)
    return True


# ---------------------------------------------------------------------------
# wait_for_arrival — block until drone is within tolerance of target lat/lon
# ---------------------------------------------------------------------------
def wait_for_arrival(
    node: Node,
    lat: float,
    lon: float,
    tolerance_m: float = 1.5,
    timeout: float = 60.0,
) -> bool:
    """
    Subscribe to /mavros/global_position/global and spin until the vehicle
    is within *tolerance_m* metres of the target (lat, lon).
    """
    import time
    import math
    from sensor_msgs.msg import NavSatFix

    arrived = False

    def _haversine(lat1, lon1, lat2, lon2):
        """Return distance in metres between two GPS points."""
        R = 6_371_000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def _gps_cb(msg: NavSatFix):
        nonlocal arrived
        dist = _haversine(msg.latitude, msg.longitude, lat, lon)
        if dist <= tolerance_m:
            arrived = True

    sub = node.create_subscription(
        NavSatFix,
        "/mavros/global_position/global",
        _gps_cb,
        sensor_qos,
    )

    deadline = time.time() + timeout
    while not arrived and time.time() < deadline:
        time.sleep(0.25)

    node.destroy_subscription(sub)

    if arrived:
        node.get_logger().info(
            f"[mavros_utils] Arrived at ({lat:.7f}, {lon:.7f})"
        )
    else:
        node.get_logger().warn(
            f"[mavros_utils] Arrival timeout after {timeout}s "
            f"(target: {lat:.7f}, {lon:.7f})"
        )

    return arrived
def wait_for_ready(node: Node, timeout: float = 30.0) -> bool:
    """
    Wait until MAVROS is connected and EKF is healthy enough to arm.
    Checks /mavros/state for connected=True and system_status=3 (standby).
    """
    import time
    from mavros_msgs.msg import State

    ready = False

    def _state_cb(msg: State):
        nonlocal ready
        # system_status 3 = MAV_STATE_STANDBY (ready to arm)
        if msg.connected and msg.system_status == 3:
            ready = True

    sub = node.create_subscription(State, "/mavros/state", _state_cb, sensor_qos)

    deadline = time.time() + timeout
    while not ready and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)

    node.destroy_subscription(sub)

    if ready:
        node.get_logger().info("[mavros_utils] FCU ready for arming")
    else:
        node.get_logger().error(f"[mavros_utils] FCU not ready after {timeout}s")

    return ready

# ---------------------------------------------------------------------------
# get_current_gps — read one GPS fix and return (lat, lon)
# ---------------------------------------------------------------------------
def get_current_gps(
    node: Node,
    timeout: float = 10.0,
) -> tuple | None:
    """
    Read a single message from /mavros/global_position/global and return
    (latitude, longitude) as a tuple, or None on timeout.
    """
    import time
    from sensor_msgs.msg import NavSatFix

    fix = None

    def _cb(msg: NavSatFix):
        nonlocal fix
        if msg.status.status >= 0:   # STATUS_FIX or better
            fix = (msg.latitude, msg.longitude)

    sub = node.create_subscription(
        NavSatFix,
        "/mavros/global_position/global",
        _cb,
        sensor_qos,
    )

    deadline = time.time() + timeout
    while fix is None and time.time() < deadline:
        time.sleep(0.25)

    node.destroy_subscription(sub)

    if fix:
        node.get_logger().info(
            f"[mavros_utils] Current GPS: lat={fix[0]:.7f}, lon={fix[1]:.7f}"
        )
    else:
        node.get_logger().warn("[mavros_utils] Could not get GPS fix")

    return fix
