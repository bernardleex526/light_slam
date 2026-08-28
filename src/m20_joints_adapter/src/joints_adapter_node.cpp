// Copyright 2026 admin
//
// Bridges the M20 robot's /JOINTS_DATA topic (drdds/msg/JointsData) to the
// standard /joint_states topic (sensor_msgs/msg/JointState) that the
// leg_wheel_odom node subscribes to. On the real robot /joint_states (a
// Gazebo-only topic) does not exist, so this adapter is required.

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "drdds/msg/joints_data.hpp"

class M20JointsAdapterNode : public rclcpp::Node
{
public:
  M20JointsAdapterNode()
  : Node("m20_joints_adapter")
  {
    joints_data_topic_ = this->declare_parameter("joints_data_topic", "/JOINTS_DATA");
    joint_states_topic_ = this->declare_parameter("joint_states_topic", "/joint_states");

    // The M20 robot publishes /JOINTS_DATA with reliable QoS by default.
    joints_sub_ = this->create_subscription<drdds::msg::JointsData>(
      joints_data_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const drdds::msg::JointsData::SharedPtr msg) {OnJointsData(msg);});

    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(), "m20_joints_adapter bridging '%s' -> '%s'",
      joints_data_topic_.c_str(), joint_states_topic_.c_str());
  }

private:
  void OnJointsData(const drdds::msg::JointsData::SharedPtr msg)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = rclcpp::Time(msg->header.stamp);
    js.header.frame_id = "";

    const auto & joints = msg->data.joints_data;
    js.name.reserve(joints.size());
    js.position.reserve(joints.size());
    js.velocity.reserve(joints.size());
    js.effort.reserve(joints.size());

    for (size_t i = 0; i < joints.size() && i < JOINT_NAMES.size(); ++i) {
      js.name.push_back(JOINT_NAMES[i]);
      js.position.push_back(joints[i].position);
      js.velocity.push_back(joints[i].velocity);
      js.effort.push_back(joints[i].torque);
    }

    joint_states_pub_->publish(js);
  }

  // M20 official joint index (0-15) -> leg_wheel_odom joint name mapping.
  // Index order from doc 21_关节.html; names from leg_wheel_odom/types.hpp and
  // leg_wheel_odom_node.hpp.
  //   0: LeftFrontHipX,  1: LeftFrontHipY,  2: LeftFrontKnee,  3: LeftFrontWheel
  //   4: RightFrontHipX, 5: RightFrontHipY, 6: RightFrontKnee, 7: RightFrontWheel
  //   8: LeftBackHipX,   9: LeftBackHipY,  10: LeftBackKnee,  11: LeftBackWheel
  //  12: RightBackHipX, 13: RightBackHipY, 14: RightBackKnee, 15: RightBackWheel
  static constexpr std::array<const char *, 16> JOINT_NAMES = {
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
  };

  std::string joints_data_topic_;
  std::string joint_states_topic_;
  rclcpp::Subscription<drdds::msg::JointsData>::SharedPtr joints_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<M20JointsAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
