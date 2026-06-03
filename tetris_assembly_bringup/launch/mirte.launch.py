from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	slam_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution([FindPackageShare("slam"), "launch", "slam.launch.py"])
		)
	)

	return LaunchDescription([slam_launch])
