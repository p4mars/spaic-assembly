from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def include(pkg, *path):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(pkg), *path])
            )
        )

    return LaunchDescription([
        include("slam",          "launch", "slam.launch.py"),
        include("detection",     "launch", "detection.launch.py"),
        include("orchestration", "launch", "orchestrator.launch.py"),
    ])
