#include "core/system/navigation_output.h"
#include <stdexcept>
namespace lightning {
namespace {
geometry_msgs::msg::Pose Pose(const SE3 &p) {
    geometry_msgs::msg::Pose m;
    m.position.x = p.translation().x();
    m.position.y = p.translation().y();
    m.position.z = p.translation().z();
    auto q = p.unit_quaternion();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    m.orientation.w = q.w();
    return m;
}
geometry_msgs::msg::TransformStamped Transform(const SE3 &p, const builtin_interfaces::msg::Time &stamp,
                                               const std::string &parent, const std::string &child) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    const auto pose = Pose(p);
    t.transform.translation.x = pose.position.x;
    t.transform.translation.y = pose.position.y;
    t.transform.translation.z = pose.position.z;
    t.transform.rotation = pose.orientation;
    return t;
}
}  // namespace
NavigationOutput::NavigationOutput(rclcpp::Node::SharedPtr node, const YAML::Node &config) : node_(node) {
    const auto s = config["system"];
    map_ = s["map_frame"].as<std::string>("map");
    odom_ = s["odom_frame"].as<std::string>("odom");
    base_ = s["base_frame"].as<std::string>("base_link");
    if (map_.empty() || odom_.empty() || base_.empty() || map_ == odom_ || odom_ == base_ || map_ == base_)
        throw std::runtime_error("map/odom/base frames must be nonempty and distinct");
    auto t = s["imu_from_base_translation"].as<std::vector<double>>(std::vector<double>{0, 0, 0});
    auto q = s["imu_from_base_quaternion_xyzw"].as<std::vector<double>>(std::vector<double>{0, 0, 0, 1});
    if (t.size() != 3 || q.size() != 4) throw std::runtime_error("Invalid IMU/base extrinsic dimensions");
    Quatd rotation(q[3], q[0], q[1], q[2]);
    Vec3d translation(t[0], t[1], t[2]);
    if (!translation.allFinite() || !rotation.coeffs().allFinite() || rotation.norm() < 1e-6)
        throw std::runtime_error("Invalid IMU/base extrinsic");
    imu_base_ = SE3(rotation.normalized(), translation);
    local_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(s["lio_odom_topic"].as<std::string>("/odom"), 10);
    global_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(s["odom_topic"].as<std::string>("/ODOM"), 10);
    pose_pub_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>(s["lio_pose_topic"].as<std::string>("/lio_pose"), 10);
    if (s["pub_tf"].as<bool>(true)) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}
void NavigationOutput::Correction(const SE3 &correction) {
    std::lock_guard<std::mutex> lock(mutex_);
    correction_ = correction;
    has_correction_ = true;
}
void NavigationOutput::Global(const SE3 &pose, double stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(stamp) || stamp <= last_global_ || !pose.matrix().allFinite()) return;
    // Results may arrive before their matching LIO state. Buffer, never pair
    // a historical map estimate with the latest odometry sample.
    pending_.emplace_back(stamp, pose);
    if (pending_.size() > 100) pending_.pop_front();
    MatchGlobal();
}
void NavigationOutput::MatchGlobal() {
    while (!pending_.empty() && !history_.empty()) {
        auto p = pending_.front();
        if (p.first < history_.front().first || p.first <= last_global_) {
            pending_.pop_front();
            continue;
        }
        if (p.first > history_.back().first) return;
        SE3 local = history_.front().second;
        for (size_t i = 1; i < history_.size(); ++i) {
            const auto &a = history_[i - 1], &b = history_[i];
            if (p.first <= b.first) {
                const double alpha = (p.first - a.first) / (b.first - a.first);
                local = SE3(a.second.unit_quaternion().slerp(alpha, b.second.unit_quaternion()),
                            (1 - alpha) * a.second.translation() + alpha * b.second.translation());
                break;
            }
        }
        correction_ = p.second * local.inverse();
        has_correction_ = true;
        last_global_ = p.first;
        pending_.pop_front();
    }
}
void NavigationOutput::Local(const NavState &state, const Cov &cov) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(state.timestamp_) || state.timestamp_ <= last_stamp_ || !state.GetPose().matrix().allFinite() ||
        !state.vel_.allFinite() || !cov.allFinite())
        return;
    const SE3 odom_imu = state.GetPose(), odom_base = odom_imu * imu_base_;
    Vec3d omega = Vec3d::Zero();
    double dt = state.timestamp_ - last_stamp_;
    if (!history_.empty() && dt > 0 && dt < 1) omega = (history_.back().second.so3().inverse() * state.rot_).log() / dt;
    const Vec3d velocity =
        odom_base.so3().inverse() * (state.vel_ + state.rot_ * (omega.cross(imu_base_.translation())));
    omega = imu_base_.so3().inverse() * omega;
    history_.emplace_back(state.timestamp_, odom_imu);
    last_stamp_ = state.timestamp_;
    while (history_.size() > 200) history_.pop_front();
    MatchGlobal();
    nav_msgs::msg::Odometry m;
    m.header.stamp = rclcpp::Time(static_cast<int64_t>(state.timestamp_ * 1e9));
    m.header.frame_id = odom_;
    m.child_frame_id = base_;
    m.pose.pose = Pose(odom_base);
    m.twist.twist.linear.x = velocity.x();
    m.twist.twist.linear.y = velocity.y();
    m.twist.twist.linear.z = velocity.z();
    m.twist.twist.angular.x = omega.x();
    m.twist.twist.angular.y = omega.y();
    m.twist.twist.angular.z = omega.z();
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();
    J.block<3, 3>(0, 3) = -state.rot_.matrix() * SO3::hat(imu_base_.translation());
    // ROS pose covariance uses fixed axes in the header frame.
    J.block<3, 3>(3, 3) = state.rot_.matrix();
    const Eigen::Matrix<double, 6, 6> pose_cov = J * cov.topLeftCorner<6, 6>() * J.transpose();
    const Mat3d velocity_cov =
        odom_base.rotationMatrix().transpose() * cov.block<3, 3>(6, 6) * odom_base.rotationMatrix();
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) m.pose.covariance[i * 6 + j] = pose_cov(i, j);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m.twist.covariance[i * 6 + j] = velocity_cov(i, j);
    for (int i = 3; i < 6; ++i) m.twist.covariance[i * 6 + i] = dt > 0 && dt < 1 ? 2 * cov(i, i) / (dt * dt) : 1e6;
    local_pub_->publish(m);
    if (tf_) tf_->sendTransform(Transform(odom_base, m.header.stamp, odom_, base_));
    if (has_correction_) {
        m.header.frame_id = map_;
        m.pose.pose = Pose(correction_ * odom_base);
        // Rotate both fixed-axis position and orientation errors into map.
        Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
        A.topLeftCorner<3, 3>() = correction_.rotationMatrix();
        A.bottomRightCorner<3, 3>() = correction_.rotationMatrix();
        const Eigen::Matrix<double, 6, 6> global_cov = A * pose_cov * A.transpose();
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) m.pose.covariance[i * 6 + j] = global_cov(i, j);
        global_pub_->publish(m);
        geometry_msgs::msg::PoseStamped p;
        p.header = m.header;
        p.pose = m.pose.pose;
        pose_pub_->publish(p);
        if (tf_) tf_->sendTransform(Transform(correction_, m.header.stamp, map_, odom_));
    }
}
}  // namespace lightning
