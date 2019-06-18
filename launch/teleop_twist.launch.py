from launch.actions import SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    keystroke_node = Node(
        package='keystroke', node_executable='keystroke_listen', output='screen', node_name='keystroke_listen',
        parameters=[{'exit_on_esc': True}], arguments=['__log_level:=warn'])
    twist_node = Node(
        package='keystroke', node_executable='keystroke_arrows_to_twist', output='screen', node_name='arrows_to_twist',
        parameters=[{'publish_period': 0.1, 'linear_scale': 0.1, 'angular_scale': 0.2}])
    return LaunchDescription([
        keystroke_node,
        twist_node,
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=keystroke_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=twist_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])
