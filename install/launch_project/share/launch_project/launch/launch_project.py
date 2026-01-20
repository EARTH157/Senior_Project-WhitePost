import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='motor_control',
            executable='stepmotor1',
            name='stepmotor1',
        ),
        launch_ros.actions.Node(
            package='motor_control',
            executable='stepmotor2',
            name='stepmotor2',
        ),
        launch_ros.actions.Node(
            package='motor_control',
            executable='stepmotor3',
            name='stepmotor3',
        ),
        launch_ros.actions.Node(
            package='motor_control',
            executable='servodrive',
            name='servodrive',
        ),
    ])