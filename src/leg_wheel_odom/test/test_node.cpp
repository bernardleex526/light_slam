// Copyright 2026 admin
#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "leg_wheel_odom/leg_wheel_odom_node.hpp"

namespace
{

void EnsureRclInit()
{
  static std::once_flag flag;
  std::call_once(
    flag, []() {
      rclcpp::init(0, nullptr);
      std::atexit([]() {rclcpp::shutdown();});
    });
}

template<class Predicate>
bool SpinUntil(
  const rclcpp::Node::SharedPtr & harness, const rclcpp::Node::SharedPtr & dut,
  std::chrono::milliseconds timeout, Predicate done)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(harness);
    rclcpp::spin_some(dut);
    if (done()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

sensor_msgs::msg::JointState MakeJointState(const std::vector<double> & wheel_vel)
{
  sensor_msgs::msg::JointState js;
  js.header.stamp = rclcpp::Clock().now();
  js.name = {
    "fl_wheel_joint", "hl_wheel_joint", "fr_wheel_joint", "hr_wheel_joint",
    "fl_hipy_joint", "fl_hipx_joint", "fl_knee_joint",
    "fr_hipy_joint", "fr_hipx_joint", "fr_knee_joint",
    "hl_hipy_joint", "hl_hipx_joint", "hl_knee_joint",
    "hr_hipy_joint", "hr_hipx_joint", "hr_knee_joint"};
  js.position = std::vector<double>(16, 0.0);
  js.velocity = std::vector<double>(16, 0.0);
  js.velocity[0] = wheel_vel[0];     // fl_wheel_joint
  js.velocity[1] = wheel_vel[1];     // hl_wheel_joint
  js.velocity[2] = wheel_vel[2];     // fr_wheel_joint
  js.velocity[3] = wheel_vel[3];     // hr_wheel_joint
  return js;
}

}  // namespace

TEST(OdomNodeTest, WheelModeStraight)
{
  EnsureRclInit();
  auto harness = std::make_shared<rclcpp::Node>("test_harness_wheel_straight");
  auto dut = std::make_shared<LegWheelOdomNode>();

  auto js_pub = harness->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  auto imu_pub = harness->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  unsigned int odom_count = 0;
  nav_msgs::msg::Odometry odom;
  auto odom_sub = harness->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_wheel", 10,
    [&](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      ++odom_count;
      odom = *msg;
    });

  sensor_msgs::msg::Imu imu;    // 零加速度 → imu_vx_ 保持 0，不打滑路径
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 0.0;
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  imu_pub->publish(imu);

  auto js = MakeJointState({10.0, 10.0, 10.0, 10.0});
  const bool received = SpinUntil(
    harness, dut, std::chrono::milliseconds(3000),
    [&]() {
      js.header.stamp = rclcpp::Clock().now();
      js_pub->publish(js);
      return odom_count > 0;
    });

  ASSERT_TRUE(received) << "no /odom_wheel message within 3 s";
  EXPECT_NEAR(odom.twist.twist.linear.x, 0.5, 1e-3);       // (wl+wr)/2 * r
  EXPECT_NEAR(odom.twist.twist.angular.z, 0.0, 1e-3);      // 等速直行 → ωz = 0
  EXPECT_NEAR(odom.twist.covariance[0], 0.02 * 0.02, 1e-6);
  EXPECT_NEAR(odom.twist.covariance[35], 0.02 * 0.02, 1e-6);
}

TEST(OdomNodeTest, DifferentialTurn)
{
  EnsureRclInit();
  auto harness = std::make_shared<rclcpp::Node>("test_harness_diff_turn");
  auto dut = std::make_shared<LegWheelOdomNode>();

  auto js_pub = harness->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  unsigned int odom_count = 0;
  nav_msgs::msg::Odometry odom;
  auto odom_sub = harness->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_wheel", 10,
    [&](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      ++odom_count;
      odom = *msg;
    });

  auto js = MakeJointState({9.0, 9.0, 11.0, 11.0});
  const bool received = SpinUntil(
    harness, dut, std::chrono::milliseconds(3000),
    [&]() {
      js.header.stamp = rclcpp::Clock().now();
      js_pub->publish(js);
      return odom_count > 0;
    });

  ASSERT_TRUE(received) << "no /odom_wheel message within 3 s";
  EXPECT_NEAR(odom.twist.twist.linear.x, 0.5, 1e-3);       // (9+11)/2 * r
  EXPECT_NEAR(odom.twist.twist.angular.z, 2.0 * 0.05 / 0.137, 1e-3);    // (wr-wl)*r/d
}

TEST(OdomNodeTest, NoJointStatesNoPublish)
{
  EnsureRclInit();
  auto harness = std::make_shared<rclcpp::Node>("test_harness_no_js");
  auto dut = std::make_shared<LegWheelOdomNode>();

  unsigned int odom_count = 0;
  auto odom_sub = harness->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_wheel", 10,
    [&](const nav_msgs::msg::Odometry::ConstSharedPtr) {++odom_count;});

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(harness);
    rclcpp::spin_some(dut);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(odom_count, 0u);    // last_js_time_.sec == 0 守卫 → 50Hz 定时器 ~30 次全部跳过
}
