#pragma once
#include <deque>
#include <vector>
#include "common/eigen_types.h"
#include "common/odom.h"
#include "core/lio/eskf.hpp"
namespace lightning {
// Planar increment in the previous IMU frame. State errors: world position,
// right-multiplicative attitude. Weight is inverse observation variance.
void BuildWheelObs(const SE3 &pred_inc, const SE3 &meas_inc, double weight, ESKF::CustomObservationModel &obs,
                   const SO3 &previous_rotation = SO3());
// Integrate body-frame twist only over fully bracketed, fresh measurements.
std::vector<OdomPtr> SelectWheelSamples(std::deque<OdomPtr> &buffer, double start, double end);

bool IntegrateWheelTwist(const std::vector<OdomPtr> &samples, double start, double end, SE3 &increment,
                         double max_gap = 0.15);
}  // namespace lightning
