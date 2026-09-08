#include "core/lio/wheelspeed_obs.h"
#include <algorithm>
#include <cmath>

namespace lightning {

void BuildWheelObs(const SE3 &pred_inc, const SE3 &meas_inc, double weight, ESKF::CustomObservationModel &obs,
                   const SO3 &previous_rotation) {
    if (!std::isfinite(weight) || weight <= 0) {
        obs.valid_ = false;
        return;
    }
    // Translation increments are in the previous IMU frame; position errors in
    // NavState are world-additive. Attitude errors are right multiplicative.
    const Vec3d rp = meas_inc.translation() - pred_inc.translation();
    const Vec3d rr = (pred_inc.so3().inverse() * meas_inc.so3()).log();
    const double angle = rr.norm();
    const Mat3d hat = SO3::hat(rr);
    const double a = angle < 1e-5 ? 1.0 / 12.0 : (1.0 - 0.5 * angle / std::tan(0.5 * angle)) / (angle * angle);
    const Mat3d left_inverse = Mat3d::Identity() - 0.5 * hat + a * hat * hat;
    Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
    J.block<2, 3>(0, 0) = previous_rotation.matrix().transpose().topRows<2>();
    J.block<1, 3>(2, 3) = left_inverse.row(2);
    const Vec3d residual(rp.x(), rp.y(), rr.z());
    obs.HTH_ += weight * J.transpose() * J;
    obs.HTr_ += weight * J.transpose() * residual;
    obs.lidar_residual_mean_ += residual.squaredNorm();
    obs.valid_ = true;
}

// Preserve the boundary sample and all future arrivals for the next interval.
std::vector<OdomPtr> SelectWheelSamples(std::deque<OdomPtr> &buffer, double start, double end) {
    while (buffer.size() > 1 && buffer[1]->timestamp_ <= start) buffer.pop_front();
    std::vector<OdomPtr> selected;
    for (const auto &sample : buffer) {
        selected.push_back(sample);
        if (sample->timestamp_ >= end) break;
    }
    return selected;
}

bool IntegrateWheelTwist(const std::vector<OdomPtr> &input, double start, double end, SE3 &increment, double max_gap) {
    if (!std::isfinite(start) || !std::isfinite(end) || end <= start || end - start > 1.0 || max_gap <= 0 ||
        input.size() < 2)
        return false;
    auto samples = input;
    for (const auto &o : samples) {
        if (!o || !std::isfinite(o->timestamp_) || !o->linear.allFinite() || !o->angular.allFinite()) return false;
    }
    std::sort(samples.begin(), samples.end(),
              [](const OdomPtr &a, const OdomPtr &b) { return a->timestamp_ < b->timestamp_; });
    if (samples.front()->timestamp_ > start || samples.back()->timestamp_ < end) return false;
    SE3 result;
    double covered = start;
    for (size_t i = 1; i < samples.size(); ++i) {
        const auto &a = samples[i - 1];
        const auto &b = samples[i];
        if (b->timestamp_ <= a->timestamp_) return false;
        const double lo = std::max(start, a->timestamp_), hi = std::min(end, b->timestamp_);
        if (hi <= lo) continue;
        const double gap = b->timestamp_ - a->timestamp_;
        if (gap > max_gap || lo > covered + 1e-9) return false;
        const double alpha = ((lo + hi) * 0.5 - a->timestamp_) / gap;
        const Vec3d v = (1 - alpha) * a->linear + alpha * b->linear;
        const double wz = (1 - alpha) * a->angular.z() + alpha * b->angular.z();
        const double dt = hi - lo, yaw = wz * dt;
        const double sinc = std::abs(yaw) < 1e-8 ? dt : std::sin(yaw) / wz;
        const double cosc = std::abs(yaw) < 1e-8 ? 0.5 * wz * dt * dt : (1 - std::cos(yaw)) / wz;
        result = result * SE3(SO3::rotZ(yaw), Vec3d(sinc * v.x() - cosc * v.y(), cosc * v.x() + sinc * v.y(), 0));
        covered = hi;
    }
    if (covered < end - 1e-9) return false;
    increment = result;
    return true;
}
}  // namespace lightning
