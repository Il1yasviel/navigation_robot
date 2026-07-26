import re
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _create_map_saver(context):
    name = LaunchConfiguration('map_name').perform(context)
    output_dir = Path(LaunchConfiguration('output_dir').perform(context)).expanduser()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
        return [LogInfo(msg='ERROR: map_name may contain only letters, digits, _ and -'),
                EmitEvent(event=Shutdown(reason='invalid map name'))]
    target = output_dir / name
    if target.with_suffix('.yaml').exists() or target.with_suffix('.pgm').exists():
        return [LogInfo(msg=f'ERROR: map already exists: {target}'),
                EmitEvent(event=Shutdown(reason='refusing to overwrite map'))]
    output_dir.mkdir(parents=True, exist_ok=True)
    return [Node(
        package='nav2_map_server', executable='map_saver_cli', name='map_saver_cli',
        arguments=['-f', str(target)],
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
        }],
        output='screen')]


def generate_launch_description():
    default_dir = [EnvironmentVariable('HOME'), '/.ros/navigation_robot/maps']
    return LaunchDescription([
        DeclareLaunchArgument('map_name', description='New map basename'),
        DeclareLaunchArgument('output_dir', default_value=default_dir),
        OpaqueFunction(function=_create_map_saver),
    ])
