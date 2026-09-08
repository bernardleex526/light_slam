#include "core/lio/laser_mapping.h"
#include "core/lio/wheelspeed_obs.h"
namespace lightning {
void LaserMapping::WheelSpeedObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    obs.valid_ = false;
    SE3 measurement;
    if (prev_frame_time_ <= 0 ||
        !IntegrateWheelTwist(std::vector<OdomPtr>(measures_.odom_.begin(), measures_.odom_.end()), prev_frame_time_,
                             measures_.lidar_end_time_, measurement))
        return;
    const SE3 prediction = prev_frame_pose_.inverse() * s.GetPose();
    const double weight = wheel_odom_weight_ * (last_frame_degenerate_ ? wheel_degeneracy_boost_ : 1.0);
    BuildWheelObs(prediction, measurement, weight, obs, prev_frame_pose_.so3());
}
}  // namespace lightning
