import launch_ros.actions
from launch.actions import SetEnvironmentVariable

import launch


def generate_launch_description():
    return launch.LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        launch_ros.actions.Node(
            package='keystroke', node_executable='keystroke_listen', output='screen', node_name='keystroke_listen',
            parameters=[{'exit_on_esc': True}]),
    ])
