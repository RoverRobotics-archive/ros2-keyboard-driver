import launch
import launch_ros.actions


def generate_launch_description():
    keystroke_node = launch_ros.actions.Node(
        package='keystroke', node_executable='keystroke_listen', output='screen', node_name='keystroke_listen',
        parameters=[{'exit_on_esc': True}], arguments=['__log_level:=warn'])
    twist_node = launch_ros.actions.Node(
        package='keystroke', node_executable='arrow_keys_to_twist', output='screen', node_name='keystroke_to_twist',
        parameters=[{'publish_period': 0.1}])
    return launch.LaunchDescription([
        keystroke_node,
        twist_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=keystroke_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=twist_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
