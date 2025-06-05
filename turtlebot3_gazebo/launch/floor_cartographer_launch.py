import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
    rviz_config_file = os.path.join(turtlebot3_cartographer_dir, 'rviz', 'cartographer.rviz')
    world_path = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'floor.world')
    map_save_path = os.path.expanduser('~/maps/my_floor_map')

    # Launch Gazebo with the desired world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn the TurtleBot3 using the provided launch file
    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'robot_name': 'turtlebot3_waffle'}.items()
    )

    # Start Cartographer SLAM
    cartographer_node = Node(
        package='turtlebot3_cartographer',
        executable='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', os.path.join(turtlebot3_cartographer_dir, 'configuration_files'),
            '-configuration_basename', 'turtlebot3_waffle.lua'
        ],
        remappings=[('/scan', '/scan')]
    )

    # Occupancy grid node to build the map
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/scan', '/scan')]
    )

    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Save the map automatically when the system shuts down
    map_saver_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_save_path],
        output='screen'
    )

    # Trigger map save on shutdown
    shutdown_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[map_saver_cmd]
        )
    )

    return LaunchDescription([
        gazebo,
        spawn_turtlebot3,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
        shutdown_handler
    ])

