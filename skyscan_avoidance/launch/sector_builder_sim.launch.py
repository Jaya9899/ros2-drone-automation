# sector_builder_sim.launch.py — sector builder against Gazebo depth.
#
# Bridges the Gazebo depth camera into ROS, runs sector_builder_node on it, and
# publishes the static camera TF. As in Stage 1, obstacle_path selects the
# MAVROS-plugin path (default) or the pymavlink fallback.
#
# NOTE: Gazebo depth is perfect — noiseless and never dropping out on blank
# walls — so Gates 3 and 4 CANNOT be validated here (they are bench/hardware
# gates). Exercise the coverage guard in sim with sim_dropout_fraction instead.
#
#   ros2 launch skyscan_avoidance sector_builder_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('skyscan_avoidance')
    config = os.path.join(pkg, 'config', 'avoidance.yaml')
    path = LaunchConfiguration('obstacle_path').perform(context)
    mavlink_url = LaunchConfiguration('mavlink_url').perform(context)
    gz_depth = LaunchConfiguration('gz_depth_topic').perform(context)
    gz_info = LaunchConfiguration('gz_camera_info_topic').perform(context)

    nodes = []

    # gz -> ROS bridge for depth Image + CameraInfo (canonical definition in
    # gz_depth_bridge.launch.py so there is one source of truth for the mapping).
    bridge_launch = os.path.join(pkg, 'launch', 'gz_depth_bridge.launch.py')
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch),
        launch_arguments={'depth_topic': gz_depth,
                          'camera_info_topic': gz_info}.items(),
    ))

    builder_remaps = [
        ('~/depth', gz_depth),
        ('~/camera_info', gz_info),
    ]
    # /avoidance/scan is never remapped onto the MAVROS plugin (mirror bug).
    nodes.append(Node(
        package='skyscan_avoidance',
        executable='sector_builder_node',
        name='sector_builder_node',
        output='screen',
        parameters=[config],
        remappings=builder_remaps,
    ))

    # Our own publisher injects a correct OBSTACLE_DISTANCE (reversed, BODY_FRD).
    transport = 'mavros' if path == 'sink' else 'pymavlink'
    nodes.append(Node(
        package='skyscan_avoidance',
        executable='obstacle_publisher_node',
        name='obstacle_publisher_node',
        output='screen',
        parameters=[config, {'transport': transport,
                             'mavlink_url': mavlink_url}],
    ))

    # Coverage guard runs alongside the builder, consuming its scan + coverage.
    nodes.append(Node(
        package='skyscan_avoidance',
        executable='coverage_guard_node',
        name='coverage_guard_node',
        output='screen',
        parameters=[config],
    ))

    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_forward_tf',
        # Named flags: the old-style positional form (x y z yaw pitch roll
        # frame child) is deprecated and warns on Jazzy.
        arguments=['--x', '0.1', '--y', '0.0', '--z', '0.0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'oak_forward'],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('obstacle_path', default_value='sink',
                              choices=['sink', 'pymavlink']),
        DeclareLaunchArgument('mavlink_url',
                              default_value='udpout:127.0.0.1:14551'),
        DeclareLaunchArgument('gz_depth_topic', default_value='/oak/depth'),
        DeclareLaunchArgument('gz_camera_info_topic',
                              default_value='/oak/depth/camera_info'),
        OpaqueFunction(function=_setup),
    ])
