#pragma once
#include "common/eigen_types.h"
#include "core/lio/eskf.hpp"

namespace lightning {

/// 轮速/腿式里程计的帧间位姿增量观测模型
/// 约束 x, y, yaw 三个自由度；z/roll/pitch 行恒为 0（交给退化感知处理）
/// @param pred_inc 预测增量（当前状态 ⊖ 上帧状态）
/// @param meas_inc 轮速里程计测得增量（T_odom(t0)^{-1} * T_odom(t1)）
/// @param weight   观测权重（= 1/R 等效缩放）
void BuildWheelObs(const SE3 &pred_inc, const SE3 &meas_inc, double weight, ESKF::CustomObservationModel &obs);

}  // namespace lightning
