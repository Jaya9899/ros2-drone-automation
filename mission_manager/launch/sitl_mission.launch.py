# sitl_mission.launch.py — SITL-specific launch file for end-to-end testing
#
# Launches the full mission stack configured for ArduPilot SITL:
#   - MAVROS (optional — set launch_mavros:=true)
#   - Safety monitor
#   - Path planner
#   - Payload node
#   - Telemetry aggregator
#   - SITL QR simulator (replaces real camera)
#   - Mission node (delayed 8s for SITL + MAVROS stabilization)
#
# Does NOT launch OAK-D (not available in SITL).
#
# Usage:
#   ros2 launch mission_manager sitl_mission.launch.py
#   ros2 launch mission_manager sitl_mission.launch.py launch_mavros:=true

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ----- Paths -----
    mission_mgr_share = get_package_share_directory("mission_manager")
    mission_params = os.path.join(
        mission_mgr_share, "config", "mission_params.yaml")

    # ----- Launch arguments -----
    use_sim_arg = DeclareLaunchArgument(
        "use_sim", default_value="true",
        description="Use simulation time",
    )
    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros", default_value="false",
        description="Launch MAVROS (set false if started separately)",
    )
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="tcp://127.0.0.1:5760",
        description="MAVROS FCU URL (SITL tcp, or serial device:baud for hardware)",
    )

    # ================================================================
    # MAVROS (optional — usually started separately with SITL)
    # ================================================================
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("mavros"), "launch", "apm.launch"
            ])
        ]),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
        }.items(),
        condition=IfCondition(
            LaunchConfiguration("launch_mavros")
        ),
    )

    # ================================================================
    # Safety monitor (always first)
    # ================================================================
    safety_monitor = Node(
        package="mission_manager",
        executable="safety_monitor",
        name="safety_monitor",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # Path planner (direct MAVROS waypoint navigation)
    # ================================================================
    path_planner_node = Node(
        package="path_planner",
        executable="path_planner_node",
        name="path_planner_node",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # Payload node
    # ================================================================
    payload_node = Node(
        package="payload",
        executable="payload_node",
        name="payload_node",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # Telemetry aggregator
    # ================================================================
    telemetry_node = Node(
        package="mission_manager",
        executable="telemetry_node",
        name="telemetry_node",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # SITL QR simulator (replaces real camera in simulation)
    # ================================================================
    sitl_qr_sim = Node(
        package="mission_manager",
        executable="sitl_qr_simulator",
        name="sitl_qr_simulator",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # QR Scanner Node (perception — provides /perception/scan_qr service)
    # In SITL, the sitl_qr_simulator handles /qr/detection directly,
    # but the QR scanner node provides the services the FSM expects.
    # ================================================================
    qr_scanner_node = Node(
        package="perception",
        executable="qr_scanner_node",
        name="qr_scanner_node",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # Mission node (delayed 8s to let SITL + MAVROS stabilize)
    # ================================================================
    mission_node = Node(
        package="mission_manager",
        executable="mission_node",
        name="mission_node",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # Assemble launch description
    # ================================================================
    return LaunchDescription([
        use_sim_arg,
        launch_mavros_arg,
        fcu_url_arg,

        LogInfo(msg="[SITL] Starting mission stack for SITL testing..."),

        # MAVROS (optional)
        mavros_launch,

        # Core nodes (start immediately)
        safety_monitor,
        payload_node,
        path_planner_node,
        telemetry_node,
        sitl_qr_sim,
        qr_scanner_node,

        # Mission node — delayed to let everything stabilize
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="[SITL] Launching mission_node after 8s delay..."),
                mission_node,
            ],
        ),
    ])
