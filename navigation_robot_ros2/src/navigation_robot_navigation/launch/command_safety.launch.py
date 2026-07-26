import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('navigation_robot_navigation'), 'config')
    mux_default = os.path.join(config_dir, 'twist_mux.yaml')
    collision_default = os.path.join(config_dir, 'collision_monitor.yaml')
    mux_config = LaunchConfiguration('twist_mux_config')
    collision_config = LaunchConfiguration('collision_monitor_config')
    return LaunchDescription([
        DeclareLaunchArgument('twist_mux_config', default_value=mux_default),
        DeclareLaunchArgument('collision_monitor_config', default_value=collision_default),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[mux_config],
            remappings=[('cmd_vel_out', '/cmd_vel_muxed')],
            output='screen',
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            parameters=[collision_config],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            parameters=[{
                'autostart': True,
                'use_sim_time': False,
                'node_names': ['collision_monitor'],
            }],
            output='screen',
        ),
    ])
