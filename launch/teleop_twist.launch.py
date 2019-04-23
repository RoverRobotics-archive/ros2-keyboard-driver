import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    keystroke_node = Node(
        package='keystroke', node_executable='keystroke_listen', output='screen', node_name='keystroke_listen',
        parameters=[{'exit_on_esc': True}], arguments=['__log_level:=warn'])
    twist_node = Node(
        package='keystroke', node_executable='keystroke_arrows_to_twist', output='screen', node_name='arrows_to_twist',
        parameters=[{'publish_period': 0.1, 'linear_scale': 1.0, 'angular_scale': 0.2}])
    return LaunchDescription([
        keystroke_node,
        twist_node,
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
