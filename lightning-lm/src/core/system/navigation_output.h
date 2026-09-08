#pragma once
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "common/nav_state.h"
namespace lightning {
// Publishes at measurement time, never wall-timestamps stale states.
// T_imu_base is the pose of the base frame in the IMU frame.
class NavigationOutput {
   public:
    using Cov = Eigen::Matrix<double, 12, 12>;
    NavigationOutput(rclcpp::Node::SharedPtr node, const YAML::Node &config);
    void Local(const NavState &state, const Cov &cov);
    void Global(const SE3 &map_imu, double stamp);
    void Correction(const SE3 &map_odom);
    const SE3 &ImuFromBase() const { return imu_base_; }

   private:
    void MatchGlobal();
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_pub_, global_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::string map_, odom_, base_;
    SE3 imu_base_, correction_;
    bool has_correction_ = false;
    double last_stamp_ = -1, last_global_ = -1;
    std::deque<std::pair<double, SE3>> history_, pending_;
    std::mutex mutex_;
};
}  // namespace lightning
