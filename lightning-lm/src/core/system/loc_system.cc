//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/system/navigation_output.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_loc");

    // use_sim_time 由 yaml 控制（默认 false，真机用墙钟）
    bool use_sim_time = false;
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        use_sim_time = root["system"]["use_sim_time"].as<bool>(false);
    } catch (...) {}
    node_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    rclcpp::QoS qos(10);
    bool sensor_best_effort = false;
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        sensor_best_effort = root["system"]["sensor_best_effort"].as<bool>(false);
    } catch (...) {}
    if (sensor_best_effort) {
        qos.best_effort();
    }

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    // /ODOM 发布器（M20 planner.service 订阅 /ODOM 作为导航核心输入；话题名由
    // system.odom_topic 参数化，默认 /ODOM，隔离模式可设为 /m20_slam/odom）


    // /initialpose 订阅（支持 RViz 2D Pose Estimate 与外部重定位，见手册 §6.4）
    initialpose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            SE3 pose(Quatd(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
                     Vec3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z));
            LOG(INFO) << "received /initialpose, setting init pose";
            SetInitPose(pose);
        });

    navigation_output_ = std::make_shared<NavigationOutput>(node_, YAML::LoadFile(yaml_path));
    loc_->SetOdomCallback([this](const NavState &state, const NavigationOutput::Cov &cov) {
        navigation_output_->Local(state, cov);
    });
    loc_->SetTFCallback([this](const geometry_msgs::msg::TransformStamped &p) {
        const auto &q=p.transform.rotation; const auto &t=p.transform.translation;
        navigation_output_->Global(SE3(Quatd(q.w,q.x,q.y,q.z), Vec3d(t.x,t.y,t.z)), ToSec(p.header.stamp));
    });

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    const SE3 imu_pose = pose * navigation_output_->ImuFromBase().inverse();
    loc_->SetExternalPose(imu_pose.unit_quaternion(), imu_pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning