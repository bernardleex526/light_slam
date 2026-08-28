// Copyright 2026 admin
#pragma once
#include <Eigen/Core>
#include <array>
#include <string>

namespace leg_wheel_odom
{

using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;

struct WheelParams
{
  double wheel_radius = 0.05;     // 轮半径 m
  double track_width = 0.137;     // 轮距 m（M20 左右轮中心距）
  double max_slip_ratio = 0.3;    // 打滑阈值（相对偏差）
};

struct LegParams
{
  double hip_len = 0.06;     // 髋部到腿根
  double thigh_len = 0.25;   // 大腿长 L2（per M20 doc 21_关节.html §10.4）
  double calf_len = 0.25;    // 小腿长 L4（per M20 doc 21_关节.html §10.4）
};

inline const std::array<std::string, 2> WHEEL_LEFT_JOINTS = {"fl_wheel_joint", "hl_wheel_joint"};
inline const std::array<std::string, 2> WHEEL_RIGHT_JOINTS = {"fr_wheel_joint", "hr_wheel_joint"};

}  // namespace leg_wheel_odom
