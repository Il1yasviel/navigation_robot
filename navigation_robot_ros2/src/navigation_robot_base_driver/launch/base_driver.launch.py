from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    base_node = LifecycleNode(
        package='navigation_robot_base_driver',
        executable='base_driver_node',
        name='base_driver',
        namespace='',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'wheel_radius_m': ParameterValue(
                LaunchConfiguration('wheel_radius_m'), value_type=float),
            'wheel_separation_m': ParameterValue(
                LaunchConfiguration('wheel_separation_m'), value_type=float),
            'motion_enabled': ParameterValue(
                LaunchConfiguration('motion_enabled'), value_type=bool),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/navigation_base'),
        DeclareLaunchArgument('wheel_radius_m', default_value='0.0'),
        DeclareLaunchArgument('wheel_separation_m', default_value='0.0'),
        DeclareLaunchArgument('motion_enabled', default_value='false'),
        base_node,
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=base_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(base_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )),
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(base_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
    ])
