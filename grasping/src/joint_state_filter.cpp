#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateFilter : public rclcpp::Node {
public:
  JointStateFilter()
  : rclcpp::Node("joint_state_filter")
  {
    auto qos = rclcpp::SensorDataQoS();
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(kOutputTopic, qos);
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      kInputTopic, qos,
      std::bind(&JointStateFilter::on_joint_state, this, std::placeholders::_1));
  }

private:
  static constexpr const char * kInputTopic = "joint_states";
  static constexpr const char * kOutputTopic = "joint_states_filtered";
  static constexpr const char * kDropSuffix = "_mimic";

  bool should_drop(const std::string & name) const
  {
    constexpr size_t kDropSuffixLen = 6;
    return name.size() >= kDropSuffixLen &&
      name.compare(name.size() - kDropSuffixLen, kDropSuffixLen, kDropSuffix) == 0;
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    sensor_msgs::msg::JointState filtered;
    filtered.header = msg->header;

    filtered.name.reserve(msg->name.size());
    filtered.position.reserve(msg->position.size());
    filtered.velocity.reserve(msg->velocity.size());
    filtered.effort.reserve(msg->effort.size());

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto & name = msg->name[i];
      if (should_drop(name)) {
        continue;
      }

      filtered.name.push_back(name);

      if (i < msg->position.size()) {
        filtered.position.push_back(msg->position[i]);
      }
      if (i < msg->velocity.size()) {
        filtered.velocity.push_back(msg->velocity[i]);
      }
      if (i < msg->effort.size()) {
        filtered.effort.push_back(msg->effort[i]);
      }
    }

    publisher_->publish(filtered);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateFilter>());
  rclcpp::shutdown();
  return 0;
}
