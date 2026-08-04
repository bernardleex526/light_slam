// Copyright 2026 admin
#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>

#include "leg_wheel_odom/leg_odometry_model.hpp"
#include "leg_wheel_odom/wheel_diff_model.hpp"

using leg_wheel_odom::LegOdometryModel;
using leg_wheel_odom::LegParams;
using leg_wheel_odom::Vec2;
using leg_wheel_odom::Vec3;
using leg_wheel_odom::WHEEL_LEFT_JOINTS;
using leg_wheel_odom::WHEEL_RIGHT_JOINTS;
using leg_wheel_odom::WheelDiffModel;
using leg_wheel_odom::WheelParams;

class LegWheelOdomNode : public rclcpp::Node
{
public:
  LegWheelOdomNode()
  : Node("leg_wheel_odom")
  {
    wheel_params_.wheel_radius = this->declare_parameter("wheel_radius", 0.05);
    wheel_params_.track_width = this->declare_parameter("track_width", 0.137);
    wheel_params_.max_slip_ratio = this->declare_parameter("slip_ratio_threshold", 0.3);
    leg_params_.hip_len = this->declare_parameter("leg_params.hip_len", 0.06);
    leg_params_.thigh_len = this->declare_parameter("leg_params.thigh_len", 0.28);
    leg_params_.calf_len = this->declare_parameter("leg_params.calf_len", 0.28);
    contact_effort_th_ = this->declare_parameter("contact_effort_threshold", 5.0);
    publish_rate_ = this->declare_parameter("publish_rate", 50.0);
    auto js_topic = this->declare_parameter("joint_states_topic", "/joint_states");
    auto imu_topic = this->declare_parameter("imu_topic", "/imu");

    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      js_topic, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {OnJointStates(msg);});
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {OnImu(msg);});

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::Int8>("/odom_wheel_mode", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
      [this]() {Publish();});
  }

private:
  void OnJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_pos_[msg->name[i]] = msg->position[i];
      joint_vel_[msg->name[i]] = msg->velocity[i];
      if (!msg->effort.empty()) {joint_effort_[msg->name[i]] = msg->effort[i];}
    }
    last_js_time_ = msg->header.stamp;
  }

  void OnImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    imu_linear_.x = msg->linear_acceleration.x;
    imu_linear_.y = msg->linear_acceleration.y;
    imu_angular_ = msg->angular_velocity.z;
    imu_vx_ += msg->linear_acceleration.x * 0.02;      // 简化积分：仅用于打滑一致性粗判
  }

  double Vel(const std::map<std::string, double> & m, const std::string & name) const
  {
    auto it = m.find(name);
    return it == m.end() ? 0.0 : it->second;
  }

  int DetectMode() const
  {
    // 轮式：轮关节有速度且腿关节接近静止；腿式：腿关节在动（接触力超过阈值）
    double wl = 0, wr = 0;
    for (auto & n : WHEEL_LEFT_JOINTS) {wl += Vel(joint_vel_, n);}
    for (auto & n : WHEEL_RIGHT_JOINTS) {wr += Vel(joint_vel_, n);}
    double wheel_speed = (std::fabs(wl) + std::fabs(wr)) / 2.0;
    double leg_speed = 0.0;
    for (auto & kv : joint_vel_) {
      if (kv.first.find("wheel") == std::string::npos) {leg_speed += std::fabs(kv.second);}
    }
    if (leg_speed > 0.3) {
      return 2;                                     // 腿动 → 腿式
    }
    if (wheel_speed > 0.05) {
      return 1;                                     // 轮动 → 轮式
    }
    return mode_;                                   // 静止保持上一模式
  }

  void Publish()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (last_js_time_.sec == 0) {return;}

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = last_js_time_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    mode_ = DetectMode();
    std_msgs::msg::Int8 mode_msg;
    mode_msg.data = mode_;
    mode_pub_->publish(mode_msg);

    double cov_scale = 1.0;
    if (mode_ == 1) {
      double wl = Vel(joint_vel_, "fl_wheel_joint") + Vel(joint_vel_, "hl_wheel_joint");
      double wr = Vel(joint_vel_, "fr_wheel_joint") + Vel(joint_vel_, "hr_wheel_joint");
      Vec2 t = WheelDiffModel::Twist(wl / 2.0, wr / 2.0, wheel_params_);
      odom.twist.twist.linear.x = t[0];
      odom.twist.twist.angular.z = t[1];
      // 打滑检测：轮速体速 vs IMU 积分体速 相对偏差
      if (std::fabs(t[0]) > 0.1 && imu_vx_ != 0) {
        double ratio = std::fabs(t[0] - imu_vx_) / std::max(std::fabs(t[0]), 1e-3);
        if (ratio > wheel_params_.max_slip_ratio) {cov_scale = 10.0 * ratio;}
      }
    } else if (mode_ == 2) {
      Vec2 q(Vel(joint_pos_, "fl_hipy_joint"), Vel(joint_pos_, "fl_knee_joint"));
      Vec2 qd(Vel(joint_vel_, "fl_hipy_joint"), Vel(joint_vel_, "fl_knee_joint"));
      Vec3 v = LegOdometryModel::BodyVelocityFromStanceLeg(q, qd, leg_params_);
      odom.twist.twist.linear.x = v[0];
      odom.twist.twist.linear.z = v[1];
      // 腿式轮速不可信 → 协方差放大
      cov_scale = 100.0;
    } else {
      cov_scale = 1000.0;        // 无有效运动源，重度膨胀
    }

    for (int i = 0; i < 6; ++i) {
      double base = (i == 0 || i == 5) ? 0.02 : 0.2;        // vx/ωz 置信，其余宽松
      odom.twist.covariance[i * 6 + i] = base * base * cov_scale;
    }
    odom_pub_->publish(odom);
  }

  WheelParams wheel_params_;
  LegParams leg_params_;
  double contact_effort_th_ = 5.0;
  double publish_rate_ = 50.0;
  int mode_ = 0;
  double imu_vx_ = 0.0;
  double imu_angular_ = 0.0;
  geometry_msgs::msg::Vector3 imu_linear_;
  std::map<std::string, double> joint_pos_, joint_vel_, joint_effort_;
  builtin_interfaces::msg::Time last_js_time_;
  std::mutex mtx_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
