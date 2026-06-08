from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    declare_sim = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Use simulation time and sim robot_description",
    )

    moveit_config = (
        MoveItConfigsBuilder("mirte")
        .robot_description(
            file_path="config/mirte_master.urdf.xacro",
            mappings={"sim": sim},
        )
        .robot_description_semantic(file_path="config/mirte_master.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": sim}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=[("joint_states", "joint_states_filtered")],
    )

    joint_state_filter_node = Node(
        package="grasping",
        executable="joint_state_filter",
        name="joint_state_filter",
        output="screen",
    )

    grasping_node = Node(
        package="grasping",
        executable="grasping_node",
        name="grasping",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": sim},
        ],
        remappings=[("joint_states", "joint_states_filtered")],
    )

    return LaunchDescription([
        declare_sim,
        joint_state_filter_node,
        move_group_node,
        grasping_node,
    ])
