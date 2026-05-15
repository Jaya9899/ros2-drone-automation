# mission.launch.py — Master launch file for Aerothon UAS Mission 2
#
# Launches the full mission stack in a single command:
#   ros2 launch mission_manager mission.launch.py
#
# Replaces run_sim.sh (which opened 6 gnome-terminal windows).
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ----- Paths -----
    mission_mgr_share = get_package_share_directory("mission_manager")
    mission_params = os.path.join(mission_mgr_share, "config", "mission_params.yaml")
    nav2_params = os.path.join(mission_mgr_share, "config", "nav2_params.yaml")

    # ----- Launch arguments -----
    use_sim = DeclareLaunchArgument(
        "use_sim", default_value="true",
        description="Use simulation time"
    )
    use_nav2_arg = DeclareLaunchArgument(
        "use_nav2", default_value="true",
        description="Launch Nav2 stack for obstacle avoidance"
    )
    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros", default_value="false",
        description="Launch MAVROS (set false if started separately)"
    )

    # ================================================================
    # MAVROS (optional — usually started separately with SITL)
    # ================================================================
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("mavros"), "launch", "apm.launch.py"
            ])
        ]),
        launch_arguments={
            "fcu_url": "udp://127.0.0.1:14550@",
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
    # Nav2 stack
    # ================================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
            ])
        ]),
        launch_arguments={
            "params_file": nav2_params,
            "use_sim_time": LaunchConfiguration("use_sim"),
        }.items(),
    )

    # ================================================================
    # OAK-D camera driver (depthai-ros)
    # ================================================================
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("depthai_ros_driver"), "launch", "camera.launch.py"
            ])
        ]),
        launch_arguments={
            "name": "oak",
            "camera_model": "OAK-D",
        }.items(),
    )

    # ================================================================
    # Assemble launch description
    # ================================================================
    return LaunchDescription([
        use_sim,
        use_nav2_arg,
        launch_mavros_arg,

        # Start safety monitor first (always running)
        safety_monitor,

        # Start payload node
        payload_node,

        # Start path planner (needs Nav2 to be up for full functionality)
        path_planner_node,

        # Start Nav2 stack
        nav2_launch,

        # Start OAK-D driver
        oakd_launch,

        # Start mission node last (it begins the state machine)
        # Delay by 5 seconds to let other nodes initialize
        TimerAction(
            period=5.0,
            actions=[mission_node],
        ),
    ])
