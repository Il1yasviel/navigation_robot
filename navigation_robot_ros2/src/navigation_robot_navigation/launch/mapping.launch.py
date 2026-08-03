import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('navigation_robot_navigation')
    slam_default = os.path.join(package_dir, 'config', 'slam_toolbox.yaml')
    slam_config = LaunchConfiguration('slam_config')
    return LaunchDescription([
        DeclareLaunchArgument('slam_config', default_value=slam_default),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config],
            output='screen',
        ),
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver',
            parameters=[{
                'use_sim_time': False,
                'save_map_timeout': 5.0,
                'free_thresh_default': 0.25,
                'occupied_thresh_default': 0.65,
            }],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapping',
            parameters=[{
                'autostart': True,
                'use_sim_time': False,
                'node_names': ['map_saver'],
            }],
            output='screen',
        ),
    ])
