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

TEST(InertialPropagation, IntegratesAccelerationInWorldFrame) {
    NavState state;
    state.grav_.setZero();
    for (int i = 0; i < 100; ++i) {
        state.oplus(state.get_f(Vec3d::Zero(), Vec3d(1, 0, 0)), 0.01);
    }
    EXPECT_NEAR(state.vel_.x(), 1.0, 1e-10);
    EXPECT_NEAR(state.pos_.x(), 0.495, 1e-10);
}

TEST(Fusion, InvalidWheelPreservesLidarUpdate) {
    ESKF::Options options;
    options.epsi_.setConstant(1e-6);
    options.lidar_obs_func_ = [](NavState &s, ESKF::CustomObservationModel &obs) {
        obs.valid_ = true;
        obs.HTH_.setIdentity();
        obs.HTr_.setZero();
        obs.HTr_[0] = 0.1 - s.pos_.x();
        obs.lidar_residual_mean_ = std::abs(obs.HTr_[0]);
    };
    options.wheelspeed_obs_func_ = [](NavState &, ESKF::CustomObservationModel &obs) { obs.valid_ = false; };
    ESKF lidar, combined;
    lidar.Init(options);
    combined.Init(options);
    lidar.Update(ESKF::ObsType::LIDAR, 1.0);
    combined.Update(ESKF::ObsType::WHEEL_SPEED_AND_LIDAR, 1.0);
    EXPECT_GT(lidar.GetX().pos_.x(), 0.0);
    EXPECT_NEAR(combined.GetX().pos_.x(), lidar.GetX().pos_.x(), 1e-12);
    EXPECT_LT((combined.GetP() - lidar.GetP()).norm(), 1e-12);
}

TEST(WheelObs, RotatedWorldPositionCorrection) {
    ESKF::CustomObservationModel obs;
    obs.HTH_.setZero();
    obs.HTr_.setZero();
    BuildWheelObs(SE3(), SE3(SO3(), Vec3d(1, 0, 0)), 1, obs, SO3::rotZ(M_PI / 2));
    EXPECT_NEAR(obs.HTr_[0], 0, 1e-10);
    EXPECT_NEAR(obs.HTr_[1], 1, 1e-10);
}

TEST(WheelObs, JacobianMatchesStatePerturbation) {
    const SO3 previous = SO3::exp(Vec3d(.2, -.1, 1.3));
    const SE3 pred(SO3::exp(Vec3d(.1, .3, .2)), Vec3d(.3, .2, .1));
    const SE3 meas(SO3::exp(Vec3d(-.2, .1, .4)), Vec3d(.6, .5, .1));
    auto residual = [&meas](const SE3 &x) {
        Vec3d rp = meas.translation() - x.translation();
        return Vec3d(rp.x(), rp.y(), (x.so3().inverse() * meas.so3()).log().z());
    };
    Eigen::Matrix<double, 3, 6> jac;
    const double eps = 1e-6;
    for (int j = 0; j < 6; ++j) {
        Vec3d d = Vec3d::Zero();
        d[j % 3] = eps;
        SE3 plus = pred, minus = pred;
        if (j < 3) {
            plus.translation() += previous.inverse() * d;
            minus.translation() -= previous.inverse() * d;
        } else {
            plus.so3() = pred.so3() * SO3::exp(d);
            minus.so3() = pred.so3() * SO3::exp(-d);
        }
        jac.col(j) = -(residual(plus) - residual(minus)) / (2 * eps);
    }
    ESKF::CustomObservationModel obs;
    obs.HTH_.setZero();
    obs.HTr_.setZero();
    BuildWheelObs(pred, meas, 1, obs, previous);
    EXPECT_LT((obs.HTH_ - jac.transpose() * jac).norm(), 1e-8);
    EXPECT_LT((obs.HTr_ - jac.transpose() * residual(pred)).norm(), 1e-8);
}

TEST(WheelObs, IntegratesExactIntervalAndRejectsMissingData) {
    std::vector<OdomPtr> data;
    for (int i = 0; i <= 10; ++i) {
        auto o = std::make_shared<Odom>();
        o->timestamp_ = i * .1;
        o->linear = Vec3d(1, 0, 0);
        o->angular = Vec3d(0, 0, 1);
        data.push_back(o);
    }
    SE3 inc;
    ASSERT_TRUE(IntegrateWheelTwist(data, .05, .95, inc));
    EXPECT_NEAR(inc.translation().x(), std::sin(.9), 1e-10);
    EXPECT_NEAR(inc.translation().y(), 1 - std::cos(.9), 1e-10);
    EXPECT_FALSE(IntegrateWheelTwist({data[0]}, 0, .1, inc));
    EXPECT_FALSE(IntegrateWheelTwist(data, -.1, .5, inc));
    EXPECT_FALSE(IntegrateWheelTwist({data.front(), data.back()}, 0, 1, inc));
}

#include "common/keyframe.h"
TEST(KeyframeFrames, AppliesLidarExtrinsicExactlyOnce) {
    CloudPtr cloud(new PointCloudType);
    PointType p;
    p.x = 1;
    p.y = 0;
    p.z = 0;
    cloud->push_back(p);
    NavState state;
    state.pos_ = Vec3d(10, 0, 0);
    Keyframe k(0, cloud, state, SE3(SO3::rotZ(M_PI / 2), Vec3d(0, 2, 0)));
    auto q = k.GetCloud()->front();
    EXPECT_NEAR(q.x, 0, 1e-6);
    EXPECT_NEAR(q.y, 3, 1e-6);
    EXPECT_NEAR((k.GetLIOPose() * Vec3d(q.x, q.y, q.z)).x(), 10, 1e-6);
    EXPECT_FLOAT_EQ(cloud->front().x, 1);
}

#include <thread>
#include "core/system/navigation_output.h"
TEST(NavigationFrames, MeasurementTimeExtrinsicTwistAndDelayedCorrection) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigation_output_test");
    auto cfg = YAML::Load("system: {pub_tf: false, imu_from_base_translation: [1, 0, 0]}");
    std::vector<nav_msgs::msg::Odometry> local, global;
    auto ls = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [&](nav_msgs::msg::Odometry::SharedPtr m) { local.push_back(*m); });
    auto gs = node->create_subscription<nav_msgs::msg::Odometry>(
        "/ODOM", 10, [&](nav_msgs::msg::Odometry::SharedPtr m) { global.push_back(*m); });
    NavigationOutput output(node, cfg);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    NavState state;
    state.timestamp_ = 10;
    state.rot_ = SO3::rotZ(M_PI / 2);
    state.vel_ = Vec3d(0, 2, 0);
    output.Local(state, NavigationOutput::Cov::Identity());
    state.timestamp_ = 10.1;
    state.pos_ = Vec3d(0, .2, 0);
    output.Local(state, NavigationOutput::Cov::Identity());
    // Delayed correction at t=10 must be paired with LIO at t=10, not t=10.1.
    output.Global(SE3(SO3::rotZ(M_PI / 2), Vec3d(5, 0, 0)), 10);
    state.timestamp_ = 10.2;
    state.pos_ = Vec3d(0, .4, 0);
    output.Local(state, NavigationOutput::Cov::Identity());
    output.Local(state, NavigationOutput::Cov::Identity());  // duplicate rejected
    for (int i = 0; i < 30 && (local.size() < 3 || global.empty()); ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(local.size(), 3u);
    EXPECT_EQ(global.size(), 1u);
    if (!local.empty() && !global.empty()) {
        EXPECT_EQ(local.back().header.frame_id, "odom");
        EXPECT_EQ(local.back().child_frame_id, "base_link");
        EXPECT_EQ(local.back().header.stamp.sec, 10);
        EXPECT_NEAR(local.back().header.stamp.nanosec, 200000000, 2);
        EXPECT_NEAR(local.back().pose.pose.position.y, 1.4, 1e-9);
        EXPECT_NEAR(local.back().twist.twist.linear.x, 2, 1e-9);
        EXPECT_NEAR(global.back().pose.pose.position.x, 5, 1e-9);
        EXPECT_NEAR(global.back().pose.pose.position.y, 1.4, 1e-9);
    }
    rclcpp::shutdown();
}

TEST(WheelObs, ConsecutiveScanIntervalsRetainBoundaryAndFutureSamples) {
    std::deque<OdomPtr> buffer;
    for (int i = 0; i < 5; ++i) {
        auto o = std::make_shared<Odom>();
        o->timestamp_ = i * .1;
        o->linear = Vec3d(1, 0, 0);
        o->angular = Vec3d::Zero();
        buffer.push_back(o);
    }
    SE3 first, second;
    EXPECT_TRUE(IntegrateWheelTwist(SelectWheelSamples(buffer, .05, .15), .05, .15, first));
    EXPECT_TRUE(IntegrateWheelTwist(SelectWheelSamples(buffer, .15, .25), .15, .25, second));
    EXPECT_NEAR(first.translation().x(), .1, 1e-10);
    EXPECT_NEAR(second.translation().x(), .1, 1e-10);
    EXPECT_DOUBLE_EQ(buffer.front()->timestamp_, .1);
    EXPECT_DOUBLE_EQ(buffer.back()->timestamp_, .4);
}
