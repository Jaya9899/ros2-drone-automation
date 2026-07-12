import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    world_path = os.path.join(
        get_package_share_directory('bcd_sim'),
        'worlds', 'arena.sdf'
    )

    return LaunchDescription([

        # Launch Gazebo with arena world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),

        # Launch PX4 SITL
        ExecuteProcess(
            cmd=['bash', '-c',
                 'cd ~/PX4-Autopilot && '
                 'PX4_SYS_AUTOSTART=4001 '
                 'PX4_GZ_MODEL=x500 '
                 './build/px4_sitl_default/bin/px4'],
            output='screen'
        ),

        # MAVROS — delayed to let PX4 start
        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'mavros', 'mavros_node',
                     '--ros-args',
                     '-p', 'fcu_url:=udp://:14540@127.0.0.1:14580',
                     '-p', 'gcs_url:=udp://@127.0.0.1:14550',
                     '-p', 'system_id:=1',
                     '-p', 'component_id:=191'],
                output='screen'
            )
        ]),

        # BCD Planner — delayed to let MAVROS connect
        TimerAction(period=15.0, actions=[
            Node(
                package='bcd_sim',
                executable='bcd_planner',
                name='bcd_planner',
                output='screen'
            )
        ]),

        # Mission Executor
        TimerAction(period=16.0, actions=[
            Node(
                package='bcd_sim',
                executable='mission_executor',
                name='mission_executor',
                output='screen'
            )
        ]),

        # RViz2 for visualisation
        TimerAction(period=17.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen'
            )
        ]),

    ])
