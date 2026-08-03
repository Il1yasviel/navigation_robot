import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = {
    'wheel_radius': '0.05',
    'wheel_separation': '0.25',
    'chassis_front': '0.20',
    'chassis_rear': '0.10',
    'chassis_left': '0.15',
    'chassis_right': '0.15',
    'chassis_height': '0.08',
    'lidar_x': '0', 'lidar_y': '0', 'lidar_z': '0.12',
    'lidar_roll': '0', 'lidar_pitch': '0', 'lidar_yaw': '0',
    'camera_x': '0', 'camera_y': '0', 'camera_z': '0',
    'camera_roll': '0', 'camera_pitch': '0', 'camera_yaw': '0',
    'imu_x': '0', 'imu_y': '0', 'imu_z': '0',
    'imu_roll': '0', 'imu_pitch': '0', 'imu_yaw': '0',
}


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('navigation_robot_description'),
        'urdf', 'navigation_robot.urdf.xacro')
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in ARGUMENTS.items()
    ]
    xacro_args = []
    for name in ARGUMENTS:
        xacro_args.extend([' ', name, ':=', LaunchConfiguration(name)])
    description = ParameterValue(
        Command(['xacro ', xacro_file, *xacro_args]), value_type=str)
    node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': description}],
        output='screen',
    )
    return LaunchDescription([*declarations, node])
