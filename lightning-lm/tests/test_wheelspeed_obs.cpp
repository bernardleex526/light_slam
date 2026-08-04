#include <gtest/gtest.h>
#include "core/lio/wheelspeed_obs.h"

using namespace lightning;

TEST(WheelObs, ConstraintStructure) {
    // 纯 x 平移增量：只约束 x 与 yaw
    SE3 pred(SO3(), Vec3d(1.0, 0.0, 0.0));
    SE3 meas(SO3(), Vec3d(1.0, 0.0, 0.0));
    ESKF::CustomObservationModel obs;
    obs.HTH_.setZero();
    obs.HTr_.setZero();
    BuildWheelObs(pred, meas, 1.0, obs);

    EXPECT_DOUBLE_EQ(obs.HTH_(0, 0), 1.0);  // x 行有约束
    EXPECT_DOUBLE_EQ(obs.HTH_(1, 1), 1.0);  // y 行有约束（零残差仍有信息阵占位）
    EXPECT_DOUBLE_EQ(obs.HTH_(2, 2), 0.0);  // z 无约束
    EXPECT_DOUBLE_EQ(obs.HTH_(3, 3), 0.0);  // roll 无约束
    EXPECT_DOUBLE_EQ(obs.HTH_(4, 4), 0.0);  // pitch 无约束
    EXPECT_DOUBLE_EQ(obs.HTH_(5, 5), 1.0);  // yaw 有约束
    EXPECT_LT(obs.HTr_.norm(), 1e-9);       // 零残差
}

TEST(WheelObs, ResidualSign) {
    // 测量比预测多走了 0.5m → 残差方向正确（r = z - pred，正方向）
    SE3 pred(SO3(), Vec3d(1.0, 0.0, 0.0));
    SE3 meas(SO3(), Vec3d(1.5, 0.0, 0.0));
    ESKF::CustomObservationModel obs;
    obs.HTH_.setZero();
    obs.HTr_.setZero();
    BuildWheelObs(pred, meas, 1.0, obs);
    EXPECT_GT(obs.HTr_(0), 0.0);
}

TEST(WheelObs, WeightScaling) {
    SE3 pred(SO3(), Vec3d(1.0, 0.0, 0.0));
    SE3 meas(SO3(), Vec3d(1.5, 0.0, 0.0));
    ESKF::CustomObservationModel a, b;
    a.HTH_.setZero();
    a.HTr_.setZero();
    b.HTH_.setZero();
    b.HTr_.setZero();
    BuildWheelObs(pred, meas, 1.0, a);
    BuildWheelObs(pred, meas, 4.0, b);
    EXPECT_NEAR(b.HTH_(0, 0), 4.0 * a.HTH_(0, 0), 1e-9);
}
