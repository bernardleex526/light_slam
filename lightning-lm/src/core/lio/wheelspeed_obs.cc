#include "core/lio/wheelspeed_obs.h"

namespace lightning {

void BuildWheelObs(const SE3 &pred_inc, const SE3 &meas_inc, double weight, ESKF::CustomObservationModel &obs) {
    // 残差：r = meas_inc ⊖ pred_inc（SE3 切线空间，6 维）
    const Vec3d r_pos = pred_inc.so3().matrix().transpose() * (meas_inc.translation() - pred_inc.translation());
    const Vec3d r_rot = (pred_inc.so3().inverse() * meas_inc.so3()).log();

    // 只约束 x, y, yaw；z/roll/pitch 行置零
    Eigen::Matrix<double, 6, ESKF::pose_obs_dim_> J = Eigen::Matrix<double, 6, 6>::Zero();
    J(0, 0) = 1.0;  // x <- 平移 x
    J(1, 1) = 1.0;  // y <- 平移 y
    J(5, 5) = 1.0;  // yaw <- 旋转 z

    Eigen::Matrix<double, 6, 1> r = Eigen::Matrix<double, 6, 1>::Zero();
    r[0] = r_pos[0];
    r[1] = r_pos[1];
    r[5] = r_rot[2];

    obs.HTH_ = J.transpose() * J * weight;
    obs.HTr_ = J.transpose() * r * weight;
    obs.lidar_residual_mean_ += r.squaredNorm();  // 供收敛判断（与 lidar 残差共用接口）
}

}  // namespace lightning
