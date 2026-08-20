# sector_builder_hw.launch.py — sector builder against the real OAK-D.
#
# Runs sector_builder_node on the depthai_ros_driver topics and publishes the
# static camera TF. It does NOT start the DepthAI driver — that is a separate,
# already-installed package (depthai_ros_driver v2.9.0); launch it on its own so
# the two lifecycles stay independent. Depth arrives as 16UC1 (millimetres) on
# hardware; the builder handles the units from the encoding.
#
# The driver MUST be started with our params file. Its defaults publish 1280x720
# intrinsics alongside a 640x480 depth image on this camera, which silently
# corrupts every bearing the builder computes — see config/oak_d_lite_depth.yaml.
#
#   # terminal 1: the driver (its own launch, our params)
#   ros2 launch depthai_ros_driver camera.launch.py \
#     params_file:=$(ros2 pkg prefix skyscan_avoidance)/share/skyscan_avoidance/config/oak_d_lite_depth.yaml
#   # terminal 2: our stack
#   ros2 launch skyscan_avoidance sector_builder_hw.launch.py
#
# Sanity-check the result without RViz or a vehicle:
#   python3 scripts/bench_scan_monitor.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('skyscan_avoidance')
    config = os.path.join(pkg, 'config', 'avoidance.yaml')
    path = LaunchConfiguration('obstacle_path').perform(context)
    mavlink_url = LaunchConfiguration('mavlink_url').perform(context)
    depth_topic = LaunchConfiguration('depth_topic').perform(context)
    info_topic = LaunchConfiguration('camera_info_topic').perform(context)

    nodes = []

    # /avoidance/scan is never remapped onto the MAVROS plugin (mirror bug); it
    # stays canonical for RViz/sector_viz/guard.
    builder_remaps = [
        ('~/depth', depth_topic),
        ('~/camera_info', info_topic),
    ]
    nodes.append(Node(
        package='skyscan_avoidance',
        executable='sector_builder_node',
        name='sector_builder_node',
        output='screen',
        parameters=[config],
        remappings=builder_remaps,
    ))

    # Our own publisher injects a correct OBSTACLE_DISTANCE. On hardware the
    # single UART means transport=mavros (/uas1/mavlink_sink); pymavlink opens a
    # 2nd connection (SITL bench only).
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
        arguments=['0.1', '0.0', '0.0', '0', '0', '0',
                   'base_link', 'oak_forward'],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('obstacle_path', default_value='sink',
                              choices=['sink', 'pymavlink']),
        DeclareLaunchArgument('mavlink_url',
                              default_value='udpout:127.0.0.1:14551'),
        # depthai_ros_driver default depth + info topics (400P stereo).
        DeclareLaunchArgument('depth_topic',
                              default_value='/oak/stereo/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/oak/stereo/camera_info'),
        OpaqueFunction(function=_setup),
    ])
