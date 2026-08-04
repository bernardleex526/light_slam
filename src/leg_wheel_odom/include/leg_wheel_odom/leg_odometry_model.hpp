// Copyright 2026 admin
#pragma once
#include "leg_wheel_odom/types.hpp"

namespace leg_wheel_odom
{

/// 平面 2-DoF 腿（hipy 前后摆 + knee 膝）：支撑腿足端零速 → 体速度 = -J(q)·q̇
/// 约定：髋在原点，大腿沿 +x，膝关节角 q_knee（0 = 伸直）
struct LegOdometryModel
{
  /// @param q     [hipy, knee] 当前关节角 (rad)
  /// @param q_dot [hipy_dot, knee_dot] 关节角速度 (rad/s)
  /// @return body 系速度 [vx, vz, omega_y]
  static Vec3 BodyVelocityFromStanceLeg(const Vec2 & q, const Vec2 & q_dot, const LegParams & p)
  {
    const double c1 = std::cos(q[0]), s1 = std::sin(q[0]);
    const double c2 = std::cos(q[0] + q[1]), s2 = std::sin(q[0] + q[1]);
    const double l2 = p.thigh_len, l3 = p.calf_len;

    // 足端位置（髋系）：px = l2*c1 + l3*c2, pz = l2*s1 + l3*s2
    // 雅可比 J = d p / d q（2x2，行 = [x, z]，列 = [hipy, knee]）
    Eigen::Matrix2d J;
    J(0, 0) = -l2 * s1 - l3 * s2;
    J(0, 1) = -l3 * s2;
    J(1, 0) = l2 * c1 + l3 * c2;
    J(1, 1) = l3 * c2;

    const Vec2 v_foot = J * q_dot;      // 足端速度（髋系）
    // 零速约束：体速度 = -v_foot（髋系 = body 系，平面内）
    Vec3 v;
    v[0] = -v_foot[0];
    v[1] = -v_foot[1];
    v[2] = 0.0;      // 平面模型不输出俯仰角速度
    return v;
  }
};

}  // namespace leg_wheel_odom
