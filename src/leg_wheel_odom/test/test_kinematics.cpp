// Copyright 2026 admin
#include <gtest/gtest.h>
#include "leg_wheel_odom/wheel_diff_model.hpp"
#include "leg_wheel_odom/leg_odometry_model.hpp"

using leg_wheel_odom::LegOdometryModel;
using leg_wheel_odom::LegParams;
using leg_wheel_odom::WheelDiffModel;
using leg_wheel_odom::WheelParams;

TEST(WheelDiff, StraightLine) {
  WheelParams p{0.05, 0.137, 0.3};
  auto t = WheelDiffModel::Twist(10.0, 10.0, p);    // 10 rad/s 双轮
  EXPECT_NEAR(t[0], 0.5, 1e-6);     // vx = w * r
  EXPECT_NEAR(t[1], 0.0, 1e-6);     // wz = 0
}

TEST(WheelDiff, Turn) {
  WheelParams p{0.05, 0.137, 0.3};
  auto t = WheelDiffModel::Twist(9.0, 11.0, p);    // 差速 2 rad/s
  EXPECT_NEAR(t[0], 0.5, 1e-6);     // vx = (wl+wr)/2 * r
  EXPECT_NEAR(t[1], 2.0 * 0.05 / 0.137, 1e-6);    // wz = (wr-wl)*r/d
}

TEST(LegOdom, StandingStance) {
  LegParams p{0.06, 0.28, 0.28};
  // 腿直立：hipy=0.7rad, knee=1.4rad（几何上足端接地），关节静止 → 体速度为零
  auto v = LegOdometryModel::BodyVelocityFromStanceLeg({0.7, 1.4}, {0.0, 0.0}, p);
  EXPECT_NEAR(v[0], 0.0, 1e-9);
  EXPECT_NEAR(v[1], 0.0, 1e-9);
}

TEST(LegOdom, SwingConsistency) {
  LegParams p{0.06, 0.28, 0.28};
  // 单腿关节运动，足端固定（零速约束）→ 体速度 = -J q̇，抽查非零且量级合理
  auto v = LegOdometryModel::BodyVelocityFromStanceLeg({0.7, 1.4}, {0.1, -0.1}, p);
  EXPECT_LT(v.norm(), 0.6);    // 0.1rad/s 关节速度 → 体速度应在 cm~dm/s 量级
}
