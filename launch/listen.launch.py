import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='keystroke', node_executable='keystroke_listen', output='screen', node_name='keystroke_listen',
            parameters=[{'exit_on_esc': True}]),
    ])
