from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='move_to_server',
            name='move_to_server',
            output='screen',
        ),
    ])
