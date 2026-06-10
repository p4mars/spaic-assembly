from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orchestration',
            executable='orchestrator',
            name='orchestrator',
            # Keep it in the foreground so the retry/skip prompts are visible
            # and the keyboard works.
            output='screen',
            emulate_tty=True,
        ),
    ])
