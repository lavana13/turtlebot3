from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os

def generate_launch_description():
    # Launch the gmapping SLAM node
    gmapping_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[{
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_update_interval': 5.0
        }],
        remappings=[('scan', 'scan')]
    )

    # Save the map automatically after 3 minutes (180 seconds)
    save_map = TimerAction(
        period=180.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-f', os.path.expanduser('~/ros_ws2/map/floor_map')
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gmapping_node,
        save_map
    ])



