#include "core/lio/laser_mapping.h"
#include "core/lio/wheelspeed_obs.h"

namespace lightning {

void LaserMapping::WheelSpeedObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    if (measures_.odom_.empty()) {
        obs.valid_ = false;
        return;
    }

    // 找 scan 区间 [lidar_begin, lidar_end] 两侧最近的 odom
    OdomPtr odom_t0 = nullptr, odom_t1 = nullptr;
    double best_dt0 = 1e9, best_dt1 = 1e9;
    for (auto &o : measures_.odom_) {
        double dt0 = std::fabs(o->timestamp_ - measures_.lidar_begin_time_);
        double dt1 = std::fabs(o->timestamp_ - measures_.lidar_end_time_);
        if (dt0 < best_dt0) { best_dt0 = dt0; odom_t0 = o; }
        if (dt1 < best_dt1) { best_dt1 = dt1; odom_t1 = o; }
    }
    if (!odom_t0 || !odom_t1) {
        obs.valid_ = false;
        return;
    }
    if (best_dt0 > 0.15 || best_dt1 > 0.15) {  // 时间戳不对齐则跳过
        obs.valid_ = false;
        return;
    }

    // 测量增量：T0^{-1} * T1
    //
    // leg_wheel_odom 只发布 twist（pose 恒为 identity），不能用 pose 增量
    // （identity → 观测强制"帧间不动"，把状态钉死在原地）。改用 twist 构造
    // 测量增量：Δx = vx*dt，Δyaw = wz*dt（车体坐标系，与 pred_inc 同为
    // 上一帧坐标系下的增量，近似一致）。
    const double dt = odom_t1->timestamp_ - odom_t0->timestamp_;
    if (dt <= 0.0 || dt > 1.0) {
        obs.valid_ = false;
        return;
    }
    const Vec3d v = 0.5 * (odom_t0->linear + odom_t1->linear);
    const double wz = 0.5 * (odom_t0->angular[2] + odom_t1->angular[2]);
    const SE3 meas_inc = SE3(SO3::rotZ(wz * dt), Vec3d(v[0] * dt, v[1] * dt, 0.0));

    // 预测增量：当前状态 ⊖ 上帧状态
    const SE3 pred_inc = prev_frame_pose_.inverse() * s.GetPose();

    // 权重：退化帧增强轮速约束
    double weight = wheel_odom_weight_;
    if (last_frame_degenerate_) {
        weight *= wheel_degeneracy_boost_;
    }

    BuildWheelObs(pred_inc, meas_inc, weight, obs);
}

}  // namespace lightning
