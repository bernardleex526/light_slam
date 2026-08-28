#pragma once

#include <cmath>
#include <limits>

namespace lightning {

struct MonotonicTimeResult {
    bool accepted = true;
    bool clamped = false;
    double timestamp = 0.0;
    double rollback = 0.0;
};

// M20 官方 bag 中少量 RoboSense 帧的 header.stamp 会回退约 3 ms。
// 小回退前推到上一帧之后的最小可表示时间，避免丢弃整帧；超过 20 ms
// 视为真实时钟异常并拒绝，防止掩盖传感器或系统时间复位。
inline MonotonicTimeResult NormalizeLidarTimestamp(
    double current, double last, double max_clamp_rollback = 0.020) {
    MonotonicTimeResult result;
    result.timestamp = current;
    if (!std::isfinite(current)) {
        result.accepted = false;
        return result;
    }
    if (!std::isfinite(last) || last <= 0.0 || current > last) {
        return result;
    }

    result.rollback = last - current;
    if (result.rollback > max_clamp_rollback) {
        result.accepted = false;
        return result;
    }

    result.clamped = true;
    result.timestamp = std::nextafter(last, std::numeric_limits<double>::infinity());
    return result;
}

}  // namespace lightning
