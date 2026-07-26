import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('navigation_robot_navigation')
    nav2_dir = get_package_share_directory('nav2_bringup')
    params_default = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    params = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    managed_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the map YAML file'),
        DeclareLaunchArgument('params_file', default_value=params_default),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': params,
                'use_sim_time': 'false',
                'autostart': 'true',
                'use_composition': 'false',
                'use_respawn': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'command_safety.launch.py')),
        ),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', parameters=[params], output='screen',
            remappings=[('cmd_vel', '/cmd_vel_nav_raw')],
        ),
        Node(
            package='nav2_smoother', executable='smoother_server',
            name='smoother_server', parameters=[params], output='screen',
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', parameters=[params], output='screen',
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', parameters=[params], output='screen',
            remappings=[('cmd_vel', '/cmd_vel_nav_raw')],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', parameters=[params], output='screen',
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', parameters=[params], output='screen',
            remappings=[
                ('cmd_vel', '/cmd_vel_nav_raw'),
                ('cmd_vel_smoothed', '/cmd_vel_nav'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation_robot', output='screen',
            parameters=[{
                'autostart': True,
                'use_sim_time': False,
                'node_names': managed_nodes,
            }],
        ),
    ])
