import os
from pathlib import Path

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import Shutdown, matches_action
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.actions import LifecycleNode, Node
from lifecycle_msgs.msg import Transition


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _pose_arguments(prefix, xyz, rpy):
    if not isinstance(xyz, list) or len(xyz) != 3 or not isinstance(rpy, list) or len(rpy) != 3:
        raise ValueError(f'{prefix} pose must have three xyz and three rpy values')
    return {
        f'{prefix}_x': str(float(xyz[0])),
        f'{prefix}_y': str(float(xyz[1])),
        f'{prefix}_z': str(float(xyz[2])),
        f'{prefix}_roll': str(float(rpy[0])),
        f'{prefix}_pitch': str(float(rpy[1])),
        f'{prefix}_yaw': str(float(rpy[2])),
    }


def _advanced_file(configured, default_name):
    if configured:
        return str(Path(configured).expanduser())
    return os.path.join(
        get_package_share_directory('navigation_robot_navigation'),
        'config', default_name)


def _camera_actions(camera):
    driver = camera.get('driver', 'pending')
    launch_file = camera.get('launch_file', '')
    if driver == 'pending' or not launch_file:
        return [LogInfo(msg='WARNING: camera requested but model/launch_file is still pending; continuing without camera')]
    try:
        if driver == 'orbbec':
            share = get_package_share_directory('orbbec_camera')
            source = PythonLaunchDescriptionSource(os.path.join(share, 'launch', launch_file))
            arguments = {
                'camera_name': camera.get('camera_name', 'front_camera'),
                'enable_color': str(camera.get('enable_color', True)).lower(),
                'enable_depth': str(camera.get('enable_depth', True)).lower(),
                'enable_point_cloud': str(camera.get('enable_point_cloud', False)).lower(),
                'enable_colored_point_cloud': 'false',
            }
        elif driver == 'astra':
            share = get_package_share_directory('astra_camera')
            source = AnyLaunchDescriptionSource(os.path.join(share, 'launch', launch_file))
            arguments = {
                'camera_name': camera.get('camera_name', 'front_camera'),
                'enable_color': str(camera.get('enable_color', True)).lower(),
                'enable_depth': str(camera.get('enable_depth', True)).lower(),
                'enable_point_cloud': str(camera.get('enable_point_cloud', False)).lower(),
            }
        else:
            return [LogInfo(msg=f'WARNING: unsupported camera driver "{driver}"; continuing without camera')]
    except PackageNotFoundError:
        return [LogInfo(msg=f'WARNING: camera package for "{driver}" is not installed; continuing without camera')]
    return [IncludeLaunchDescription(source, launch_arguments=arguments.items())]


def _launch_setup(context):
    config_path = Path(LaunchConfiguration('config_file').perform(context)).expanduser()
    if not config_path.is_file():
        return [LogInfo(msg=f'ERROR: config file not found: {config_path}'),
                EmitEvent(event=Shutdown(reason='missing robot config'))]
    with config_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    system = config['system']
    geometry = config['geometry']
    base = config['base']
    lidar = config['lidar']
    camera = config['camera']
    navigation = config['navigation']

    mode_override = LaunchConfiguration('operation_mode').perform(context).strip()
    mode = mode_override or system.get('operation_mode', 'sensors')
    if mode not in ('sensors', 'mapping', 'navigation'):
        return [LogInfo(msg=f'ERROR: unsupported operation_mode: {mode}'),
                EmitEvent(event=Shutdown(reason='invalid operation mode'))]

    use_camera_override = LaunchConfiguration('use_camera').perform(context).strip()
    use_camera = _as_bool(use_camera_override) if use_camera_override else _as_bool(system['use_camera'])
    use_rviz_override = LaunchConfiguration('use_rviz').perform(context).strip()
    use_rviz = _as_bool(use_rviz_override) if use_rviz_override else _as_bool(system['use_rviz'])

    measured_keys = (
        'wheel_radius_m', 'wheel_separation_m', 'chassis_front_m',
        'chassis_rear_m', 'chassis_left_m', 'chassis_right_m',
    )
    geometry_ready = all(float(geometry.get(key, 0.0)) > 0.0 for key in measured_keys)
    if mode != 'sensors' and (not geometry_ready or not _as_bool(system['motion_enabled'])):
        return [
            LogInfo(msg='ERROR: mapping/navigation refused: fill measured geometry and set motion_enabled=true'),
            EmitEvent(event=Shutdown(reason='motion safety lock')),
        ]

    if mode == 'navigation':
        map_path = Path(navigation.get('map', '')).expanduser()
        if not map_path.is_file():
            return [LogInfo(msg=f'ERROR: navigation map not found: {map_path}'),
                    EmitEvent(event=Shutdown(reason='missing navigation map'))]

    visual_radius = max(float(geometry['wheel_radius_m']), 0.05)
    visual_separation = max(float(geometry['wheel_separation_m']), 0.25)
    description_launch = os.path.join(
        get_package_share_directory('navigation_robot_description'),
        'launch', 'description.launch.py')
    description_arguments = {
        'wheel_radius': str(visual_radius),
        'wheel_separation': str(visual_separation),
        'chassis_front': str(max(float(geometry['chassis_front_m']), 0.20)),
        'chassis_rear': str(max(float(geometry['chassis_rear_m']), 0.10)),
        'chassis_left': str(max(float(geometry['chassis_left_m']), 0.15)),
        'chassis_right': str(max(float(geometry['chassis_right_m']), 0.15)),
        'chassis_height': str(max(float(geometry['chassis_height_m']), 0.08)),
    }
    description_arguments.update(_pose_arguments('lidar', geometry['lidar_xyz'], geometry['lidar_rpy']))
    description_arguments.update(_pose_arguments('camera', geometry['camera_xyz'], geometry['camera_rpy']))
    description_arguments.update(_pose_arguments('imu', geometry['imu_xyz'], geometry['imu_rpy']))

    base_node = LifecycleNode(
        package='navigation_robot_base_driver',
        executable='base_driver_node',
        name='base_driver',
        namespace='',
        output='screen',
        parameters=[{
            'transport': base['transport'],
            'serial_port': base['serial_port'],
            'baud_rate': int(base['baud_rate']),
            'tcp_host': base['tcp_host'],
            'tcp_port': int(base['tcp_port']),
            'wheel_radius_m': float(geometry['wheel_radius_m']),
            'wheel_separation_m': float(geometry['wheel_separation_m']),
            'max_rpm': float(base['max_rpm']),
            'max_linear_velocity': float(base['max_linear_velocity']),
            'max_angular_velocity': float(base['max_angular_velocity']),
            'motion_enabled': mode != 'sensors' and _as_bool(system['motion_enabled']),
            'command_timeout_sec': float(base['command_timeout_sec']),
            'keepalive_period_sec': float(base['keepalive_period_sec']),
            'imu_gyro_bias_calibration_enabled': _as_bool(
                base['imu_gyro_bias_calibration_enabled']),
            'imu_gyro_bias_calibration_samples': int(
                base['imu_gyro_bias_calibration_samples']),
            'imu_gyro_stationary_threshold_rad_s': float(
                base['imu_gyro_stationary_threshold_rad_s']),
            'imu_accel_norm_tolerance_m_s2': float(
                base['imu_accel_norm_tolerance_m_s2']),
            'imu_wheel_stationary_threshold_rpm': float(
                base['imu_wheel_stationary_threshold_rpm']),
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'imu_frame_id': 'imu_link',
        }],
    )

    configure_base = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(base_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate_base = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=base_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(base_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    actions = [
        LogInfo(msg=f'navigation_robot starting in {mode} mode'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments=description_arguments.items(),
        ),
        base_node,
        activate_base,
        configure_base,
        Node(
            package='navigation_robot_lidar_driver',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'port_name': lidar['port_name'],
                'baud_rate': int(lidar['baud_rate']),
                'frame_id': lidar['frame_id'],
                'scan_topic': lidar['scan_topic'],
                'range_min': float(lidar['range_min']),
                'range_max': float(lidar['range_max']),
                'inverted': _as_bool(lidar['inverted']),
                'scan_size': int(lidar['scan_size']),
            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[_advanced_file(navigation.get('ekf_params_file'), 'ekf.yaml')],
            output='screen',
            remappings=[('odometry/filtered', '/odom')],
        ),
    ]

    if use_camera:
        actions.extend(_camera_actions(camera))

    navigation_dir = get_package_share_directory('navigation_robot_navigation')
    if mode == 'mapping':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                navigation_dir, 'launch', 'mapping.launch.py')),
            launch_arguments={
                'slam_config': _advanced_file(
                    navigation.get('slam_params_file'), 'slam_toolbox.yaml'),
            }.items(),
        ))
    elif mode == 'navigation':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                navigation_dir, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'map': str(Path(navigation['map']).expanduser()),
                'params_file': _advanced_file(
                    navigation.get('nav2_params_file'), 'nav2_params.yaml'),
            }.items(),
        ))

    if use_rviz:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                navigation_dir, 'launch', 'rviz.launch.py'))))
    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('navigation_robot_bringup'),
        'config', 'robot_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('operation_mode', default_value=''),
        DeclareLaunchArgument('use_camera', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
