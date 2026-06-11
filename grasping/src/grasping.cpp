#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rcl_action/rcl_action.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "grasping/action/drop_tile.hpp"
#include "grasping/action/pick_tile.hpp"
#include "grasping/custom_4dof_ik.hpp"

namespace
{
const std::map<std::string, double> detection_pose = {
  {"shoulder_pan_joint", 0.0},
  {"shoulder_lift_joint", 0.2},
  {"elbow_joint", -1.3},
  {"wrist_joint", -1.57},
};

const std::map<std::string, double> carry_pose = {
  {"shoulder_pan_joint", 0.0},
  {"shoulder_lift_joint", -0.1},
  {"elbow_joint", -1.50},
  {"wrist_joint", -1.55},
};

// Height of the approach point straight above the tile [m].
constexpr double kApproachHeight = 0.03;
// Lowest the gripper_center is allowed to go in base_link [m]; tile positions are clamped up to this.
constexpr double kMinTileZ = 0.022;
}

class TileGrasping
{
public:
  explicit TileGrasping(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_, node_),
    ik_(tf_buffer_, {
      // "joint name", {lower_limit, upper_limit, lower_tolerance, upper_tolerance}
      {"shoulder_pan_joint", {-M_PI / 2.0, M_PI / 2.0, 0.01, 0.01}},
      {"shoulder_lift_joint", {-M_PI / 2.0, M_PI / 2.0, 0.0, 0.0}},
      {"elbow_joint", {-M_PI / 2.0, M_PI / 2.0, 0.01, 0.01}},
      {"wrist_joint", {-M_PI / 2.0, M_PI / 2.0, 0.02, 0.02}},
    }),
    arm_interface_(node_, "mirte_arm"),
    gripper_interface_(node_, "mirte_gripper"),
    action_group_(node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    RCLCPP_INFO(node_->get_logger(), "Moveit Arm Planning frame: %s", arm_interface_.getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "Moveit Gripper Planning frame: %s", gripper_interface_.getPlanningFrame().c_str());

    arm_interface_.setMaxVelocityScalingFactor(0.5);
    arm_interface_.setMaxAccelerationScalingFactor(0.5);
    arm_interface_.setPlanningTime(1.0);
    arm_interface_.setNumPlanningAttempts(3);
    arm_interface_.setGoalPositionTolerance(0.02);
    arm_interface_.setGoalOrientationTolerance(0.3);
    arm_interface_.setGoalJointTolerance(0.1);

    pick_server_ = rclcpp_action::create_server<PickTile>(
      node_,
      "pick_tile",
      std::bind(&TileGrasping::handle_pick_goal, this, _1, _2),
      std::bind(&TileGrasping::handle_pick_cancel, this, _1),
      std::bind(&TileGrasping::handle_pick_accepted, this, _1),
      rcl_action_server_get_default_options(),
      action_group_);

    drop_server_ = rclcpp_action::create_server<DropTile>(
      node_,
      "drop_tile",
      std::bind(&TileGrasping::handle_drop_goal, this, _1, _2),
      std::bind(&TileGrasping::handle_drop_cancel, this, _1),
      std::bind(&TileGrasping::handle_drop_accepted, this, _1),
      rcl_action_server_get_default_options(),
      action_group_);

    detection_pose_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "move_to_detection_pose",
      std::bind(&TileGrasping::handle_move_to_detection_pose, this, _1, _2));
  }

private:
  using PickTile = grasping::action::PickTile;
  using DropTile = grasping::action::DropTile;
  using PickGoalHandle = rclcpp_action::ServerGoalHandle<PickTile>;
  using DropGoalHandle = rclcpp_action::ServerGoalHandle<DropTile>;

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  grasping::CustomIK ik_;
  moveit::planning_interface::MoveGroupInterface arm_interface_;
  moveit::planning_interface::MoveGroupInterface gripper_interface_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp_action::Server<PickTile>::SharedPtr pick_server_;
  rclcpp_action::Server<DropTile>::SharedPtr drop_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detection_pose_service_;

  template <typename GoalHandleT, typename ResultT>
  bool cancel_if_requested(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::shared_ptr<ResultT> & result)
  {
    if (!goal_handle->is_canceling()) {
      return false;
    }
    result->success = false;
    result->message = "canceled";
    goal_handle->canceled(result);
    return true;
  }

  template <typename GoalHandleT, typename ResultT, typename FeedbackT>
  bool move_gripper(
    const std::string & target,
    const std::string & failure_message,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::shared_ptr<ResultT> & result,
    const std::shared_ptr<FeedbackT> & feedback)
  {
    if (cancel_if_requested(goal_handle, result)) { return false; }
    feedback->status = target == "open" ? "opening gripper" : "closing gripper";
    goal_handle->publish_feedback(feedback);
    gripper_interface_.setNamedTarget(target);
    if (!gripper_interface_.move()) {
      result->success = false;
      result->message = failure_message;
      goal_handle->abort(result);
      return false;
    }
    return true;
  }

  // Plan and execute a joint-space target, with feedback and error handling.
  template <typename GoalHandleT, typename ResultT, typename FeedbackT>
  bool move_to_joints(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::shared_ptr<ResultT> & result,
    const std::shared_ptr<FeedbackT> & feedback,
    const std::string & name,
    const std::map<std::string, double> & joints)
  {
    if (cancel_if_requested(goal_handle, result)) { return false; }
    feedback->status = name;
    goal_handle->publish_feedback(feedback);

    arm_interface_.setStartStateToCurrentState();
    arm_interface_.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(arm_interface_.plan(plan))) {
      result->success = false;
      result->message = "failed to plan " + name;
      goal_handle->abort(result);
      return false;
    }

    if (cancel_if_requested(goal_handle, result)) { return false; }

    if (!arm_interface_.execute(plan)) {
      result->success = false;
      result->message = "failed to execute " + name;
      goal_handle->abort(result);
      return false;
    }
    return true;
  }

  geometry_msgs::msg::PointStamped stamp_base_link(const geometry_msgs::msg::Point & p) const
  {
    geometry_msgs::msg::PointStamped ps;
    ps.header.frame_id = "base_link";
    ps.header.stamp = rclcpp::Time(0);
    ps.point = p;
    return ps;
  }

  // Solve IK for `target` (base_link); nullopt if unreachable / out of joint limits.
  // Throws tf2::TransformException on TF failure.
  std::optional<std::map<std::string, double>> solve_ik(const geometry_msgs::msg::Point & target) const
  {
    try {
      return ik_.solve(stamp_base_link(target));
    } catch (const grasping::IKUnreachable &) {
    } catch (const grasping::IKJointLimit &) {
    }  // tf2::TransformException intentionally propagates
    return std::nullopt;
  }

  bool move_to_detection_pose()
  {
    arm_interface_.setJointValueTarget(detection_pose);
    return static_cast<bool>(arm_interface_.move());
  }

  void handle_move_to_detection_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = move_to_detection_pose();
    response->message = response->success ? "ok" : "failed to move to detection pose";
  }

  // --- PickTile -----------------------------------------------------------

  rclcpp_action::GoalResponse handle_pick_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickTile::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_pick_cancel(const std::shared_ptr<PickGoalHandle> &)
  {
    arm_interface_.stop();
    gripper_interface_.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_pick_accepted(const std::shared_ptr<PickGoalHandle> goal_handle)
  {
    execute_pick(goal_handle);
  }

  void execute_pick(const std::shared_ptr<PickGoalHandle> goal_handle)
  {
    auto result = std::make_shared<PickTile::Result>();
    auto feedback = std::make_shared<PickTile::Feedback>();
    const auto goal = goal_handle->get_goal();

    // Plan every phase (IK) up-front, so we abort before any motion if some part of the pick is infeasible.
    feedback->status = "computing pick IK";
    goal_handle->publish_feedback(feedback);

    geometry_msgs::msg::PointStamped tile = goal->pos;
    tile.header.stamp = rclcpp::Time(0);

    std::optional<std::map<std::string, double>> grasp;
    try {
      auto tile_base = tf_buffer_.transform(tile, "base_link", tf2::durationFromSec(0.3)).point;
      tile_base.z = std::max(tile_base.z, kMinTileZ);
      grasp = solve_ik(tile_base);
      tile_base.z += kApproachHeight;
      // approach = solve_ik(tile_base);
    } catch (const tf2::TransformException & ex) {
      result->success = false;
      result->message = std::string("pick TF failed: ") + ex.what();
      goal_handle->abort(result);
      return;
    }

    // Check the tile itself first so its error takes precedence over the approach point.
    if (!grasp) {
      result->success = false;
      result->message = "tile itself is not reachable";
      goal_handle->abort(result);
      return;
    }
    // if (!approach) {
    //   result->success = false;
    //   result->message = "approach point above the tile is not reachable";
    //   goal_handle->abort(result);
    //   return;
    // }

    if (!move_gripper("open", "failed to open gripper", goal_handle, result, feedback)) { return; }
    // if (!move_to_joints(goal_handle, result, feedback, "approaching tile", *approach)) { return; }
    if (!move_to_joints(goal_handle, result, feedback, "lowering onto tile", *grasp)) { return; }
    if (!move_gripper("close", "failed to close gripper", goal_handle, result, feedback)) { return; }
    if (!move_to_joints(goal_handle, result, feedback, "moving to carry pose", carry_pose)) { return; }

    result->success = true;
    result->message = "ok";
    goal_handle->succeed(result);
  }

  // --- DropTile -----------------------------------------------------------

  rclcpp_action::GoalResponse handle_drop_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const DropTile::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_drop_cancel(const std::shared_ptr<DropGoalHandle> &)
  {
    arm_interface_.stop();
    gripper_interface_.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_drop_accepted(const std::shared_ptr<DropGoalHandle> goal_handle)
  {
    execute_drop(goal_handle);
  }

  void execute_drop(const std::shared_ptr<DropGoalHandle> goal_handle)
  {
    auto result = std::make_shared<DropTile::Result>();
    auto feedback = std::make_shared<DropTile::Feedback>();
    const auto goal = goal_handle->get_goal();

    // Plan every phase (IK) up-front, so we abort before any motion if some part of the drop is infeasible.
    feedback->status = "computing drop IK";
    goal_handle->publish_feedback(feedback);

    geometry_msgs::msg::PointStamped tile = goal->pos;
    tile.header.stamp = rclcpp::Time(0);

    std::optional<std::map<std::string, double>> place;
    try {
      auto tile_base = tf_buffer_.transform(tile, "base_link", tf2::durationFromSec(0.3)).point;
      tile_base.z = std::max(tile_base.z, kMinTileZ);
      place = solve_ik(tile_base);
      tile_base.z += kApproachHeight;
      // approach = solve_ik(tile_base);
    } catch (const tf2::TransformException & ex) {
      result->success = false;
      result->message = std::string("drop TF failed: ") + ex.what();
      goal_handle->abort(result);
      return;
    }

    // Check the drop position itself first so its error takes precedence over the approach point.
    if (!place) {
      result->success = false;
      result->message = "drop position itself is not reachable";
      goal_handle->abort(result);
      return;
    }
    // if (!approach) {
    //   result->success = false;
    //   result->message = "approach point above the drop position is not reachable";
    //   goal_handle->abort(result);
    //   return;
    // }

    // if (!move_to_joints(goal_handle, result, feedback, "approaching drop position", *approach)) { return; }
    if (!move_to_joints(goal_handle, result, feedback, "lowering to drop position", *place)) { return; }
    if (!move_gripper("open", "failed to open gripper", goal_handle, result, feedback)) { return; }
    if (!move_to_joints(goal_handle, result, feedback, "moving to detection pose", detection_pose)) { return; }

    result->success = true;
    result->message = "ok";
    goal_handle->succeed(result);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "grasping",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  TileGrasping actions(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
