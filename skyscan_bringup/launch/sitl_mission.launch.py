# sitl_mission.launch.py — Mission 2 stack against ArduPilot SITL
#
#   ros2 launch skyscan_bringup sitl_mission.launch.py
#   ros2 launch skyscan_bringup sitl_mission.launch.py launch_mavros:=true
#
# Same nodes as mission.launch.py minus the cameras and the OAK-D obstacle
# pipeline (none exist in SITL); sitl_qr_simulator stands in for the downward
# camera by publishing /qr/detection near a configured GPS point.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory("skyscan_bringup"), "config", "mission_params.yaml")

    # SITL has no RC pilot, no downward camera and no OAK-D, so the hardware
    # safety gates are relaxed here (and ONLY here). SITL also simulates a 3S
    # pack (12.6 V); the 4S battery thresholds would abort.
    sitl_battery = {"battery_critical_v": 10.5, "battery_warning_v": 11.1}
    mission_overrides = {
        "require_rc_switch": False,         # no pilot switch in SITL
        "allow_qr_fallback": True,          # no camera: fly with qr_fallback_id
        "qr_scan_attempts": 1,
        "qr_scan_timeout_s": 5.0,
        "require_avoidance_sensor": False,  # no OAK-D / coverage_guard
        "battery_critical_v": 10.5,
    }

    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros", default_value="false",
        description="Launch MAVROS (set false if started separately)")
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url", default_value="tcp://127.0.0.1:5760",
        description="MAVROS FCU URL (SITL tcp, or serial device:baud for hardware)")

    # MAVROS (Jazzy ships apm.launch as XML -> AnyLaunchDescriptionSource)
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ]),
        launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
        condition=IfCondition(LaunchConfiguration("launch_mavros")),
    )

    def mission_node_of(package, executable, overrides=None):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=[mission_params] + ([overrides] if overrides else []),
        )

    return LaunchDescription([
        launch_mavros_arg,
        fcu_url_arg,

        mavros_launch,
        mission_node_of("mission_manager", "safety_monitor", sitl_battery),
        mission_node_of("mission_manager", "telemetry_node"),
        mission_node_of("mission_manager", "sitl_qr_simulator"),
        mission_node_of("payload", "payload_node"),
        mission_node_of("path_planner", "path_planner_node",
                        {"require_avoidance_sensor": False}),
        # Provides /perception/scan_qr; with no camera the reference scan times
        # out and mission_node uses qr_fallback_id.
        mission_node_of("perception", "qr_scanner_node", {"scan_timeout_s": 5.0}),

        # Mission node last, after SITL + MAVROS have settled.
        TimerAction(period=8.0, actions=[
            mission_node_of("mission_manager", "mission_node", mission_overrides)]),
    ])
