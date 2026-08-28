// Copyright 2026 admin
#pragma once
#include "leg_wheel_odom/types.hpp"

namespace leg_wheel_odom
{

/// 差速轮式运动学：vx = (wl+wr)/2 * r, wz = (wr-wl)*r/d
struct WheelDiffModel
{
  static Vec2 Twist(double wl, double wr, const WheelParams & p)
  {
    Vec2 t;
    t[0] = (wl + wr) / 2.0 * p.wheel_radius;
    t[1] = (wr - wl) * p.wheel_radius / p.track_width;
    return t;
  }
};

}  // namespace leg_wheel_odom
