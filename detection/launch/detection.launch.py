from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detection_params_file = LaunchConfiguration("detection_params_file")

    declare_detaction_params_file_cmd = DeclareLaunchArgument(
        "detection_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("detection"), "config", "detection.yaml"]
        ),
        description="Full path to the detection parameter file.",
    )

    detection_node = Node(
        package='detection',
        executable='detection',
        parameters=[ParameterFile(detection_params_file, allow_substs=True)],
        name='detection',
        output='screen',
    )

    # The camera driver publishes with frame_id 'default_cam' but the URDF defines
    # it as 'gripper_camera_link', breaking the TF chain map→...→wrist→default_cam.
    # This manually links default_cam to wrist using the real offset from the URDF,
    # since the upstream fix was pushed too late to safely update the robot.
    
    # The gripper_camera_link has just recently been added: https://github.com/mirte-robot/mirte-ros-packages/blob/main/mirte_description/mirte_master_description/urdf/arm.xacro
    # It does not exist yet with our version, so we will manually create a static transform based on the values in the arm.xacro file
    
    # We basically manually append the tf default_cam to the wrist
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.027',
            '--y', '0',
            '--z', '-0.067',
            '--roll', '1.5707963267949',
            '--pitch', '0',
            '--yaw', '1.5707963267949',
            '--frame-id', 'wrist',
            '--child-frame-id', 'default_cam'
        ]
    )

    ld = LaunchDescription()
    ld.add_action(static_tf)
    ld.add_action(declare_detaction_params_file_cmd)
    ld.add_action(detection_node)
    return ld