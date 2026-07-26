import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('navigation_robot_lidar_driver'),
        'config', 'lidar.yaml')
    return LaunchDescription([
        Node(
            package='navigation_robot_lidar_driver',
            executable='lidar_node',
            name='lidar_node',
            parameters=[config],
            output='screen',
        )
    ])
