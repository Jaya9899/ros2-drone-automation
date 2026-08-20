# gz_depth_bridge.launch.py — bridge the Gazebo OAK-D depth camera into ROS.
#
# Section 6. Bridges the gz depth Image + CameraInfo produced by the
# oak_depth_sensor SDF block (gazebo/oak_depth_sensor.sdf) onto ROS topics the
# sector builder consumes as ~/depth and ~/camera_info.
#
# Equivalent to the spec's raw command:
#   ros2 run ros_gz_bridge parameter_bridge \
#     /oak/depth@sensor_msgs/msg/Image@gz.msgs.Image \
#     /oak/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
#
# Included by sector_builder_sim.launch.py; also runnable standalone:
#   ros2 launch skyscan_avoidance gz_depth_bridge.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_topic = LaunchConfiguration('depth_topic')
    info_topic = LaunchConfiguration('camera_info_topic')

    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/oak/depth'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/oak/depth/camera_info'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='oak_depth_bridge',
            output='screen',
            arguments=[
                [depth_topic, '@sensor_msgs/msg/Image@gz.msgs.Image'],
                [info_topic, '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            ],
        ),
    ])
