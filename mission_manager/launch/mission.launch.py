# mission.launch.py — Master launch file for Aerothon UAS Mission 2
#
# Launches the full mission stack in a single command:
#   ros2 launch mission_manager mission.launch.py
#
# Navigation uses direct MAVROS waypoint publishing.
# RL-based obstacle avoidance can be enabled via config (placeholder).
# MAVROS and SITL must still be started separately.

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ----- Paths -----
    mission_mgr_share = get_package_share_directory("mission_manager")
    mission_params = os.path.join(mission_mgr_share, "config", "mission_params.yaml")

    # ----- Launch arguments -----
    use_sim = DeclareLaunchArgument(
        "use_sim", default_value="true",
        description="Use simulation time"
    )

    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros", default_value="false",
        description="Launch MAVROS (set false if started separately)"
    )

    # SITL default. Pixhawk over USB: /dev/ttyACM0:115200
    # Pixhawk TELEM2 -> Pi 5 GPIO UART: /dev/ttyAMA0:921600
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url", default_value="tcp://127.0.0.1:5760",
        description="MAVROS FCU URL (SITL tcp, or serial device:baud for hardware)"
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
    # Mission Manager nodes
    # ================================================================
    mission_node = Node(
        package="mission_manager",
        executable="mission_node",
        name="mission_node",
        output="screen",
        parameters=[mission_params],
    )

    safety_monitor = Node(
        package="mission_manager",
        executable="safety_monitor",
        name="safety_monitor",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # Path Planner
    # ================================================================
    path_planner_node = Node(
        package="path_planner",
        executable="path_planner_node",
        name="path_planner_node",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # Payload
    # ================================================================
    payload_node = Node(
        package="payload",
        executable="payload_node",
        name="payload_node",
        output="screen",
        parameters=[mission_params],
    )

    # ================================================================
    # Perception (QR scanner + depth processor)
    # ================================================================
    qr_scanner_node = Node(
        package="perception",
        executable="qr_scanner_node",
        name="qr_scanner_node",
        output="screen",
        parameters=[mission_params],
    )

    depth_processor_node = Node(
        package="perception",
        executable="depth_processor_node",
        name="depth_processor_node",
        output="screen",
        parameters=[mission_params],
    )


    # ================================================================
    # OAK-D Lite camera driver (depthai-ros)
    # ================================================================
    # MUST use skyscan_avoidance's params file. The driver defaults publish
    # 1280x720 intrinsics next to a 640x480 depth image on the OAK-D Lite,
    # which silently mis-bins every obstacle bearing, and the default RGBD
    # pipeline's NN blob upload can crash the container. See the yaml header.
    oak_params = os.path.join(
        get_package_share_directory("skyscan_avoidance"),
        "config", "oak_d_lite_depth.yaml")
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("depthai_ros_driver"), "launch", "camera.launch.py"
            ])
        ]),
        launch_arguments={
            "name": "oak",  # must match the /oak: key in oak_params
            "camera_model": "OAK-D-LITE",
            "params_file": oak_params,
        }.items(),
    )

    # ================================================================
    # Assemble launch description
    # ================================================================
    return LaunchDescription([
        use_sim,
        launch_mavros_arg,
        fcu_url_arg,

        # MAVROS (only if launch_mavros:=true)
        mavros_launch,

        # Start safety monitor first (always running)
        safety_monitor,

        # Start payload node
        payload_node,

        # Start path planner
        path_planner_node,

        # Start perception nodes
        qr_scanner_node,
        depth_processor_node,

        # Start OAK-D driver
        oakd_launch,

        # Start mission node last (it begins the state machine)
        # Delay by 5 seconds to let other nodes initialize
        TimerAction(
            period=5.0,
            actions=[mission_node],
        ),
    ])
