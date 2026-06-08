#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>

namespace
{
const std::map<std::string, double> carry_pose = {
  {"shoulder_pan_joint", 0.0},
  {"shoulder_lift_joint", -0.1},
  {"elbow_joint", -1.50},
  {"wrist_joint", -1.55},
};

const std::map<std::string, double> grasp_pose = {
  {"shoulder_pan_joint", 0.0},
  {"shoulder_lift_joint", -1.29},
  {"elbow_joint", -0.92},
  {"wrist_joint", -0.96},
};
}

class GraspingServices
{
public:
  using Trigger = std_srvs::srv::Trigger;

  explicit GraspingServices(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    arm_interface_(node_, "mirte_arm"),
    gripper_interface_(node_, "mirte_gripper")
  {
    RCLCPP_INFO(node_->get_logger(), "Moveit Arm Planning frame: %s", arm_interface_.getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "Moveit Gripper Planning frame: %s", gripper_interface_.getPlanningFrame().c_str());

    arm_interface_.setMaxVelocityScalingFactor(0.5);
    arm_interface_.setMaxAccelerationScalingFactor(0.5);
    arm_interface_.setPlanningTime(1.0);
    arm_interface_.setNumPlanningAttempts(3);
    arm_interface_.setGoalPositionTolerance(0.02);
    arm_interface_.setGoalOrientationTolerance(0.5);
    arm_interface_.setGoalJointTolerance(0.1);

    open_gripper_service_ = node_->create_service<Trigger>(
      "open_gripper",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
        move_gripper("open", "opened gripper", "failed to open gripper", response);
      });

    close_gripper_service_ = node_->create_service<Trigger>(
      "close_gripper",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
        move_gripper("close", "closed gripper", "failed to close gripper", response);
      });

    move_to_carry_pose_service_ = node_->create_service<Trigger>(
      "move_to_carry_pose",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
        move_arm_to_carry_pose(response);
      });

    move_to_grasp_pose_service_ = node_->create_service<Trigger>(
      "move_to_grasp_pose",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
        move_arm_to_grasp_pose(response);
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface arm_interface_;
  moveit::planning_interface::MoveGroupInterface gripper_interface_;
  rclcpp::Service<Trigger>::SharedPtr open_gripper_service_;
  rclcpp::Service<Trigger>::SharedPtr close_gripper_service_;
  rclcpp::Service<Trigger>::SharedPtr move_to_carry_pose_service_;
  rclcpp::Service<Trigger>::SharedPtr move_to_grasp_pose_service_;

  bool move_gripper(
    const std::string target,
    const std::string success_message,
    const std::string failure_message,
    const std::shared_ptr<Trigger::Response> & response)
  {
    gripper_interface_.setNamedTarget(target);
    if (!gripper_interface_.move()) {
      response->success = false;
      response->message = failure_message;
      return false;
    }

    response->success = true;
    response->message = success_message;
    return true;
  }

  bool move_arm_to_carry_pose(const std::shared_ptr<Trigger::Response> & response)
  {
    arm_interface_.setJointValueTarget(carry_pose);
    if (!arm_interface_.move()) {
      response->success = false;
      response->message = "failed to move to carry pose";
      return false;
    }

    response->success = true;
    response->message = "moved to carry pose";
    return true;
  }

  bool move_arm_to_grasp_pose(const std::shared_ptr<Trigger::Response> & response)
  {
    arm_interface_.setJointValueTarget(grasp_pose);
    if (!arm_interface_.move()) {
      response->success = false;
      response->message = "failed to move to grasp pose";
      return false;
    }

    response->success = true;
    response->message = "moved to grasp pose";
    return true;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "grasping",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  GraspingServices services(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}