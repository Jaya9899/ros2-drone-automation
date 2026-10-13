# sitl_mission.launch.py — Mission 2 stack against ArduPilot SITL
#
#   ros2 launch mission_bringup sitl_mission.launch.py
#   ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true
#   ros2 launch mission_bringup sitl_mission.launch.py corridor_sim:=false
#
# Same nodes as mission.launch.py minus the cameras; sitl_qr_simulator stands
# in for the downward camera's QR detections.
#
# corridor_sim:=true (default): sitl_corridor_sim ray-casts the OAK-D depth
# and the ground markers from the SITL pose into 5 ft corridors with
# obstacles (sitl_corridor_sim section of mission_params.yaml), and the REAL
# sector_builder, coverage_guard and corridor_perception nodes run on it — the
# corridor legs are flown closed-loop, sensor health gating included.
# corridor_sim:=false: no depth at all; corridors are flown on the GPS
# centreline and the obstacle-sensor gates are off.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _stack(context, *args, **kwargs):
    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")
    avoidance_cfg = os.path.join(
        get_package_share_directory("obstacle_avoidance"), "config", "avoidance.yaml")
    sim = LaunchConfiguration("corridor_sim").perform(context) == "true"

    # SITL has no RC pilot and no downward camera, so those gates are relaxed
    # here (and ONLY here). The obstacle-sensor gates stay on when the
    # simulated depth pipeline runs.
    sensor = {"require_avoidance_sensor": sim}
    mission_overrides = {
        "require_rc_switch": False,         # no pilot switch in SITL
        "allow_qr_fallback": True,          # no camera: fly with qr_fallback_id
        "qr_scan_attempts": 1,
        "qr_scan_timeout_s": 5.0,
        "battery_critical_v": 10.5,         # SITL simulates a 3S pack (12.6 V)
        **sensor,
    }

    def node(package, executable, overrides=None, params=None):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=(params or [mission_params]) + ([overrides] if overrides else []),
        )

    nodes = [
        node("mission_manager", "sitl_qr_simulator"),
        node("payload", "payload_node"),
        node("path_planner", "path_planner_node",
             {**sensor, "require_corridor_perception": sim}),
        # Provides /perception/scan_qr; with no camera the reference scan times
        # out and mission_node uses qr_fallback_id.
        node("qr_detection", "qr_scanner_node", {"scan_timeout_s": 5.0}),
    ]
    if sim:
        remaps = [("~/depth", "/oak/stereo/image_raw"),
                  ("~/camera_info", "/oak/stereo/camera_info")]
        nodes.append(node("obstacle_avoidance", "sitl_corridor_sim"))
        # The sim renders at half resolution (render_scale 0.5): stride 2 gives
        # the same 160x120 samples the OAK-D gives at stride 4.
        # 5 Hz sim depth (hardware: 10 Hz): widen the stale cutoffs, keeping
        # the guard 0.2 s ahead of the builder watchdog as it asserts.
        timeouts = {"depth_stale_timeout_s": 0.6, "depth_watchdog_timeout_s": 0.8}
        for exe, extra in (("sector_builder_node", timeouts),
                           ("corridor_perception_node", {"corridor_stride_px": 2})):
            nodes.append(Node(package="obstacle_avoidance", executable=exe, name=exe,
                              output="screen", parameters=[avoidance_cfg, extra],
                              remappings=remaps))
        nodes.append(node("obstacle_avoidance", "coverage_guard_node", timeouts,
                          params=[avoidance_cfg]))

    # Mission node last, after SITL + MAVROS have settled.
    nodes.append(TimerAction(period=8.0, actions=[
        node("mission_manager", "mission_node", mission_overrides)]))
    return nodes


def generate_launch_description():
    # MAVROS (Jazzy ships apm.launch as XML -> AnyLaunchDescriptionSource)
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ]),
        launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
        condition=IfCondition(LaunchConfiguration("launch_mavros")),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_mavros", default_value="false",
            description="Launch MAVROS (set false if started separately)"),
        DeclareLaunchArgument(
            "fcu_url", default_value="tcp://127.0.0.1:5760",
            description="MAVROS FCU URL (SITL tcp, or serial device:baud for hardware)"),
        DeclareLaunchArgument(
            "corridor_sim", default_value="true", choices=["true", "false"],
            description="Simulated OAK-D depth + ground markers in the corridors"),
        mavros_launch,
        OpaqueFunction(function=_stack),
    ])
