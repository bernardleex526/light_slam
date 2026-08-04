# LIO 前端增强实施计划（腿/轮里程计融合 + 退化场景鲁棒性）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 lightning-lm（gaoxiang12，ROS2）上补全"车辆里程计输入"（README TODO 项 9），实现腿/轮里程计融合进 ESKF、退化方向感知增强与验证。

**Architecture:** 新包 `leg_wheel_odom` 消费 M20 关节/轮速数据输出 body 速度+协方差（nav_msgs/Odometry）；lightning-lm fork 增加 odom 输入管线（订阅→缓冲→SyncPackages→measures_.odom_），实现轮速观测函数 `WheelSpeedObsModel`（帧间位姿增量，6×6 HTH 中 z/横滚/俯仰行置 0，由内置退化感知自动处理），修复 ESKF `WHEEL_SPEED_AND_LIDAR` 分支 gap，并暴露退化信息。ESKF 核心算法（特征值分解/掩码/膨胀）不改。

**Tech Stack:** C++17、ROS2 Humble、Eigen3、PCL、gtest（ament_cmake_gtest）、Gazebo Classic 11（M20 仿真）、evo（评测）。

## Global Constraints

- 构建/运行环境：WSL Ubuntu-22.04（`/opt/ros/humble` 已装，colcon 已装）；源码根 `D:\light` = WSL `/mnt/d/light`
- 代码改动范围：`D:\light\lightning-lm\`（fork，git 仓库独立）与 `D:\light\src\leg_wheel_odom\`（新包）
- **不改 ESKF 核心算法**（`eskf.cc` 特征值分解/observable mask/P 膨胀逻辑原样），只允许：①`WHEEL_SPEED_AND_LIDAR` 分支加调一次观测函数；②新增 `degeneracy_callback_` 选项（默认空，不调用即零行为变化）
- 新增 yaml 参数一律用 `as<double>(默认值)` 形式读取，旧配置文件（无新键）必须能原样加载
- 代码风格对齐仓库：`namespace lightning`、`LOG(INFO)`、`Vec3d/Mat3d/SE3` 类型别名、行尾无空格
- 每完成一个 Task 提交一次（中文提交信息），commit 只包含本 Task 相关文件
- 构建命令统一：`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`（在 `/mnt/d/light` 下）

---

### Task 0: 环境核验与基线构建

**Files:**
- Test: 无（环境操作）

**Interfaces:**
- Produces: 可用的 colcon 工作区 `/mnt/d/light`（lightning-lm 已 clone）、基线构建产物、git 可用

- [ ] **Step 1: 核验 WSL 与 git**

Run（PowerShell）:
```powershell
wsl -d Ubuntu-22.04 -- bash -lc "source /opt/ros/humble/setup.bash && colcon --version && git --version"
```
Expected: colcon 版本号输出；git 若报 `command not found`，执行：
```bash
sudo apt update && sudo apt install -y git
```

- [ ] **Step 2: 基线构建**

Run:
```bash
cd /mnt/d/light
./lightning-lm/scripts/install_dep.sh   # 装 Pangolin/OpenCV/PCL/yaml-cpp/glog/gflags（已装会跳过/失败可忽略）
source /opt/ros/humble/setup.bash
ln -sfn lightning-lm src/lightning-lm    # 若无 src 目录先 mkdir
# 或直接在工作区根构建（colcon 发现子目录）：
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Expected: 构建成功，`install/lightning/` 存在；`source install/setup.bash && ros2 run lightning run_slam_offline --help` 能打印参数。

> 注意：lightning-lm 的 CMakeLists 是单包根目录布局，colcon 直接以 `/mnt/d/light` 为工作区、包为 `lightning`。若构建失败先解决依赖（`install_dep.sh`）。

- [ ] **Step 3: 下载数据集（离线验证用）**

从 README 链接下载转换好的数据集（OneDrive 或百度网盘 `1XmFitUtnkKa2d0YtWquQXw?pwd=xehn`），至少取：NCLT（通用回归）+ DeepRobotics M20 数据（如 demo_ysc 系列）。解压到 `/mnt/d/data/`。
Expected: `/mnt/d/data/nclt/*.db3`、`/mnt/d/data/ysc/*.db3` 存在。

- [ ] **Step 4: 基线离线建图跑通**

Run:
```bash
cd /mnt/d/light && source install/setup.bash
ros2 run lightning run_slam_offline --input_bag /mnt/d/data/ysc/<bag>.db3 --config ./lightning-lm/config/default_robosense.yaml 2>&1 | tee /tmp/baseline_ysc.log
```
Expected: 日志无 ERROR（可容忍个别 WARNING），结束后 `data/new_map/global.pcd` 与 `map.pgm` 生成；`grep -c "ERROR" /tmp/baseline_ysc.log` 为 0。

- [ ] **Step 5: 记录基线轨迹（evo 用）**

离线建图会把轨迹存到 `data/new_map/`（若无则改跑 `run_frontend_offline` 并记录 `/odom`）。将轨迹文件另存为 `/mnt/d/data/baseline_ysc_traj.tum` 备用。
Expected: 文件存在，供 Task 6 对比。

- [ ] **Step 6: Commit**

```bash
cd /mnt/d/light && git add -A && git commit -m "chore: 环境核验与基线构建（lightning-lm 基线跑通）"
```
Expected: 提交成功（仅 docs/.gitignore 会变，若 lightning-lm 仍被 ignore 则提交为空时用 `git status` 确认无异常）。

---

### Task 1: leg_wheel_odom 包骨架 + 运动学纯函数（TDD）

**Files:**
- Create: `src/leg_wheel_odom/package.xml`
- Create: `src/leg_wheel_odom/CMakeLists.txt`
- Create: `src/leg_wheel_odom/include/leg_wheel_odom/wheel_diff_model.hpp`
- Create: `src/leg_wheel_odom/include/leg_wheel_odom/leg_odometry_model.hpp`
- Create: `src/leg_wheel_odom/test/test_kinematics.cpp`
- Create: `src/leg_wheel_odom/include/leg_wheel_odom/types.hpp`

**Interfaces:**
- Produces:
  - `struct WheelParams { double wheel_radius, track_width, max_slip_ratio; }`
  - `struct LegParams { double hip_len, thigh_len, calf_len; }`（hipy=髋前后, knee=膝）
  - `namespace leg_wheel_odom { Vec2 WheelDiffModel::Twist(double wl, double wr, const WheelParams&); }`（返回 vx, wz，单位 m/s, rad/s）
  - `Vec3 LegOdometryModel::BodyVelocityFromStanceLeg(const Vec2& q_hipy, double q_knee, const Vec2& q_dot, const LegParams&);`（支撑腿平面 2-DoF 运动学，返回 body 系 vx, vz, ωy）
  - 常量：`WHEEL_LEFT_JOINTS = {"fl_wheel_joint","hl_wheel_joint"}`, `WHEEL_RIGHT_JOINTS = {"fr_wheel_joint","hr_wheel_joint"}`

- [ ] **Step 1: 写失败测试**

Create `src/leg_wheel_odom/test/test_kinematics.cpp`:
```cpp
#include <gtest/gtest.h>
#include "leg_wheel_odom/wheel_diff_model.hpp"
#include "leg_wheel_odom/leg_odometry_model.hpp"

using namespace leg_wheel_odom;

TEST(WheelDiff, StraightLine) {
    WheelParams p{0.05, 0.137, 0.3};
    auto t = WheelDiffModel::Twist(10.0, 10.0, p);  // 10 rad/s 双轮
    EXPECT_NEAR(t[0], 0.5, 1e-6);   // vx = w * r
    EXPECT_NEAR(t[1], 0.0, 1e-6);   // wz = 0
}

TEST(WheelDiff, Turn) {
    WheelParams p{0.05, 0.137, 0.3};
    auto t = WheelDiffModel::Twist(9.0, 11.0, p);  // 差速 2 rad/s
    EXPECT_NEAR(t[0], 0.5, 1e-6);   // vx = (wl+wr)/2 * r
    EXPECT_NEAR(t[1], -2.0 * 0.05 / 0.137, 1e-6);  // wz = (wr-wl)*r/d
}

TEST(LegOdom, StandingStance) {
    LegParams p{0.06, 0.28, 0.28};
    // 腿直立：hipy=0.7rad, knee=1.4rad（几何上足端接地），关节静止 → 体速度为零
    auto v = LegOdometryModel::BodyVelocityFromStanceLeg({0.7, 1.4}, {0.0, 0.0}, p);
    EXPECT_NEAR(v[0], 0.0, 1e-9);
    EXPECT_NEAR(v[1], 0.0, 1e-9);
}

TEST(LegOdom, SwingConsistency) {
    LegParams p{0.06, 0.28, 0.28};
    // 单腿关节运动，足端固定（零速约束）→ 体速度 = -J q̇，抽查非零且量级合理
    auto v = LegOdometryModel::BodyVelocityFromStanceLeg({0.7, 1.4}, {0.1, -0.1}, p);
    EXPECT_LT(v.norm(), 0.6);  // 0.1rad/s 关节速度 → 体速度应在 cm~dm/s 量级
}
```

- [ ] **Step 2: 运行确认失败**

Run: `cd /mnt/d/light && source install/setup.bash && colcon build --packages-select leg_wheel_odom --cmake-args -DCMAKE_BUILD_TYPE=Release`
Expected: 构建报错（找不到 `leg_wheel_odom/wheel_diff_model.hpp` 或包不存在）。

- [ ] **Step 3: 写实现**

Create `src/leg_wheel_odom/include/leg_wheel_odom/types.hpp`:
```cpp
#pragma once
#include <Eigen/Core>
#include <array>
#include <string>

namespace leg_wheel_odom {

using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;

struct WheelParams {
    double wheel_radius = 0.05;   // 轮半径 m
    double track_width = 0.137;   // 轮距 m（M20 左右轮中心距）
    double max_slip_ratio = 0.3;  // 打滑阈值（相对偏差）
};

struct LegParams {
    double hip_len = 0.06;   // 髋部到腿根
    double thigh_len = 0.28; // 大腿长
    double calf_len = 0.28;  // 小腿长
};

inline const std::array<std::string, 2> WHEEL_LEFT_JOINTS = {"fl_wheel_joint", "hl_wheel_joint"};
inline const std::array<std::string, 2> WHEEL_RIGHT_JOINTS = {"fr_wheel_joint", "hr_wheel_joint"};

}  // namespace leg_wheel_odom
```

Create `src/leg_wheel_odom/include/leg_wheel_odom/wheel_diff_model.hpp`:
```cpp
#pragma once
#include "leg_wheel_odom/types.hpp"

namespace leg_wheel_odom {

/// 差速轮式运动学：vx = (wl+wr)/2 * r, wz = (wr-wl)*r/d
struct WheelDiffModel {
    static Vec2 Twist(double wl, double wr, const WheelParams& p) {
        Vec2 t;
        t[0] = (wl + wr) / 2.0 * p.wheel_radius;
        t[1] = (wr - wl) * p.wheel_radius / p.track_width;
        return t;
    }
};

}  // namespace leg_wheel_odom
```

Create `src/leg_wheel_odom/include/leg_wheel_odom/leg_odometry_model.hpp`:
```cpp
#pragma once
#include "leg_wheel_odom/types.hpp"

namespace leg_wheel_odom {

/// 平面 2-DoF 腿（hipy 前后摆 + knee 膝）：支撑腿足端零速 → 体速度 = -J(q)·q̇
/// 约定：髋在原点，大腿沿 +x，膝关节角 q_knee（0 = 伸直）
struct LegOdometryModel {
    /// @param q     [hipy, knee] 当前关节角 (rad)
    /// @param q_dot [hipy_dot, knee_dot] 关节角速度 (rad/s)
    /// @return body 系速度 [vx, vz, omega_y]
    static Vec3 BodyVelocityFromStanceLeg(const Vec2& q, const Vec2& q_dot, const LegParams& p) {
        const double c1 = std::cos(q[0]), s1 = std::sin(q[0]);
        const double c2 = std::cos(q[0] + q[1]), s2 = std::sin(q[0] + q[1]);
        const double l2 = p.thigh_len, l3 = p.calf_len;

        // 足端位置（髋系）：px = l2*c1 + l3*c2, pz = l2*s1 + l3*s2
        // 雅可比 J = d p / d q（2x2，行 = [x, z]，列 = [hipy, knee]）
        Eigen::Matrix2d J;
        J(0, 0) = -l2 * s1 - l3 * s2;
        J(0, 1) = -l3 * s2;
        J(1, 0) = l2 * c1 + l3 * c2;
        J(1, 1) = l3 * c2;

        const Vec2 v_foot = J * q_dot;  // 足端速度（髋系）
        // 零速约束：体速度 = -v_foot（髋系 = body 系，平面内）
        Vec3 v;
        v[0] = -v_foot[0];
        v[1] = -v_foot[1];
        v[2] = 0.0;  // 平面模型不输出俯仰角速度
        return v;
    }
};

}  // namespace leg_wheel_odom
```

Create `src/leg_wheel_odom/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>leg_wheel_odom</name>
  <version>0.1.0</version>
  <description>Leg/Wheel odometry for wheel-legged robots (M20)</description>
  <maintainer email="dev@local">admin</maintainer>
  <license>Proprietary</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <buildtool_depend>ament_cmake_gtest</buildtool_depend>
  <buildtool_depend>eigen3_cmake_module</buildtool_depend>
  <depend>eigen</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Create `src/leg_wheel_odom/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(leg_wheel_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(leg_wheel_odom_core INTERFACE)
target_include_directories(leg_wheel_odom_core INTERFACE include)
target_link_libraries(leg_wheel_odom_core INTERFACE Eigen3::Eigen)

ament_target_dependencies(leg_wheel_odom_core INTERFACE
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics leg_wheel_odom_core)
  ament_target_dependencies(test_kinematics Eigen3::Eigen)
endif()

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_package()
```

- [ ] **Step 4: 运行确认通过**

Run: `cd /mnt/d/light && source install/setup.bash && colcon build --packages-select leg_wheel_odom && colcon test --packages-select leg_wheel_odom --event-handlers console_direct+`
Expected: 4 个测试全部 PASS（`Running 4 tests` / `[  PASSED  ] 4 tests`）。若 `StraightLine` 断言失败，检查轮距/半径单位（M20 轮半径 0.05m、轮距 0.137m 来自现有控制器配置）。

- [ ] **Step 5: Commit**

```bash
cd /mnt/d/light && git add src/leg_wheel_odom && git commit -m "feat: leg_wheel_odom 包骨架与运动学纯函数（差速/平面腿模型，单测通过）"
```

---

### Task 2: leg_wheel_odom 节点（ROS2 接线 + 打滑检测 + 模式状态机）

**Files:**
- Create: `src/leg_wheel_odom/src/leg_wheel_odom_node.cpp`
- Create: `src/leg_wheel_odom/config/leg_wheel_odom.yaml`

**Interfaces:**
- Consumes: Task 1 的 `WheelDiffModel`、`LegOdometryModel`、`WHEEL_LEFT_JOINTS/WHEEL_RIGHT_JOINTS`
- Produces: 话题 `/odom_wheel`（`nav_msgs/msg/Odometry`，body 系，`twist.covariance` 对角线有效，0-2 线速度 vx vy vz，5 角速度 wx wy wz 处填 0.01^2 基线）
- Produces: 参数（yaml）：`wheel_radius`、`track_width`、`joint_states_topic`、`imu_topic`、`publish_rate`、`slip_ratio_threshold`、`contact_effort_threshold`、`leg_params.*`
- Produces: 内部状态机输出 `/odom_wheel_mode`（`std_msgs/msg/Int8`，0=无 1=轮式 2=腿式）

- [ ] **Step 1: 写节点实现**

Create `src/leg_wheel_odom/src/leg_wheel_odom_node.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int8.hpp>
#include <map>
#include <mutex>
#include <cmath>

#include "leg_wheel_odom/wheel_diff_model.hpp"
#include "leg_wheel_odom/leg_odometry_model.hpp"

using namespace leg_wheel_odom;

class LegWheelOdomNode : public rclcpp::Node {
   public:
    LegWheelOdomNode() : Node("leg_wheel_odom") {
        wheel_params_.wheel_radius = this->declare_parameter("wheel_radius", 0.05);
        wheel_params_.track_width = this->declare_parameter("track_width", 0.137);
        wheel_params_.max_slip_ratio = this->declare_parameter("slip_ratio_threshold", 0.3);
        leg_params_.hip_len = this->declare_parameter("leg_params.hip_len", 0.06);
        leg_params_.thigh_len = this->declare_parameter("leg_params.thigh_len", 0.28);
        leg_params_.calf_len = this->declare_parameter("leg_params.calf_len", 0.28);
        contact_effort_th_ = this->declare_parameter("contact_effort_threshold", 5.0);
        publish_rate_ = this->declare_parameter("publish_rate", 50.0);
        auto js_topic = this->declare_parameter("joint_states_topic", "/joint_states");
        auto imu_topic = this->declare_parameter("imu_topic", "/imu");

        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            js_topic, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { OnJointStates(msg); });
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { OnImu(msg); });

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::Int8>("/odom_wheel_mode", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / publish_rate_)),
                                         [this]() { Publish(); });
    }

   private:
    void OnJointStates(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_pos_[msg->name[i]] = msg->position[i];
            joint_vel_[msg->name[i]] = msg->velocity[i];
            if (!msg->effort.empty()) joint_effort_[msg->name[i]] = msg->effort[i];
        }
        last_js_time_ = msg->header.stamp;
    }

    void OnImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_linear_.x = msg->linear_acceleration.x;
        imu_linear_.y = msg->linear_acceleration.y;
        imu_angular_ = msg->angular_velocity.z;
        imu_vx_ += msg->linear_acceleration.x * 0.02;  // 简化积分：仅用于打滑一致性粗判
    }

    double Vel(const std::map<std::string, double>& m, const std::string& name) const {
        auto it = m.find(name);
        return it == m.end() ? 0.0 : it->second;
    }

    int DetectMode() const {
        // 轮式：轮关节有速度且腿关节接近静止；腿式：腿关节在动（接触力超过阈值）
        double wl = 0, wr = 0;
        for (auto& n : WHEEL_LEFT_JOINTS) wl += Vel(joint_vel_, n);
        for (auto& n : WHEEL_RIGHT_JOINTS) wr += Vel(joint_vel_, n);
        double wheel_speed = (std::fabs(wl) + std::fabs(wr)) / 2.0;
        double leg_speed = 0.0;
        for (auto& kv : joint_vel_) {
            if (kv.first.find("wheel") == std::string::npos) leg_speed += std::fabs(kv.second);
        }
        if (leg_speed > 0.3) return 2;              // 腿动 → 腿式
        if (wheel_speed > 0.05) return 1;           // 轮动 → 轮式
        return mode_;                               // 静止保持上一模式
    }

    void Publish() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (last_js_time_.sec == 0) return;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = last_js_time_;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        mode_ = DetectMode();
        std_msgs::msg::Int8 mode_msg;
        mode_msg.data = mode_;
        mode_pub_->publish(mode_msg);

        double cov_scale = 1.0;
        if (mode_ == 1) {
            double wl = Vel(joint_vel_, "fl_wheel_joint") + Vel(joint_vel_, "hl_wheel_joint");
            double wr = Vel(joint_vel_, "fr_wheel_joint") + Vel(joint_vel_, "hr_wheel_joint");
            Vec2 t = WheelDiffModel::Twist(wl / 2.0, wr / 2.0, wheel_params_);
            odom.twist.twist.linear.x = t[0];
            odom.twist.twist.angular.z = t[1];
            // 打滑检测：轮速体速 vs IMU 积分体速 相对偏差
            if (std::fabs(t[0]) > 0.1 && imu_vx_ != 0) {
                double ratio = std::fabs(t[0] - imu_vx_) / std::max(std::fabs(t[0]), 1e-3);
                if (ratio > wheel_params_.max_slip_ratio) cov_scale = 10.0 * ratio;
            }
        } else if (mode_ == 2) {
            Vec2 q(Vel(joint_pos_, "fl_hipy_joint"), Vel(joint_pos_, "fl_knee_joint"));
            Vec2 qd(Vel(joint_vel_, "fl_hipy_joint"), Vel(joint_vel_, "fl_knee_joint"));
            Vec3 v = LegOdometryModel::BodyVelocityFromStanceLeg(q, qd, leg_params_);
            odom.twist.twist.linear.x = v[0];
            odom.twist.twist.linear.z = v[1];
            // 腿式轮速不可信 → 协方差放大
            cov_scale = 100.0;
        } else {
            cov_scale = 1000.0;  // 无有效运动源，重度膨胀
        }

        for (int i = 0; i < 6; ++i) {
            double base = (i == 0 || i == 5) ? 0.02 : 0.2;  // vx/ωz 置信，其余宽松
            odom.twist.covariance[i * 6 + i] = base * base * cov_scale;
        }
        odom_pub_->publish(odom);
    }

    WheelParams wheel_params_;
    LegParams leg_params_;
    double contact_effort_th_ = 5.0;
    double publish_rate_ = 50.0;
    int mode_ = 0;
    double imu_vx_ = 0.0;
    double imu_angular_ = 0.0;
    geometry_msgs::msg::Vector3 imu_linear_;
    std::map<std::string, double> joint_pos_, joint_vel_, joint_effort_;
    builtin_interfaces::msg::Time last_js_time_;
    std::mutex mtx_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mode_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegWheelOdomNode>());
    rclcpp::shutdown();
    return 0;
}
```

Create `src/leg_wheel_odom/config/leg_wheel_odom.yaml`:
```yaml
leg_wheel_odom:
  ros__parameters:
    wheel_radius: 0.05
    track_width: 0.137
    slip_ratio_threshold: 0.3
    contact_effort_threshold: 5.0
    publish_rate: 50.0
    joint_states_topic: /joint_states
    imu_topic: /imu
    leg_params:
      hip_len: 0.06
      thigh_len: 0.28
      calf_len: 0.28
```

- [ ] **Step 2: CMake 增加可执行目标**

在 `src/leg_wheel_odom/CMakeLists.txt` 的 `ament_package()` 前追加：
```cmake
add_executable(leg_wheel_odom_node src/leg_wheel_odom_node.cpp)
target_include_directories(leg_wheel_odom_node PRIVATE include)
target_link_libraries(leg_wheel_odom_node leg_wheel_odom_core)
ament_target_dependencies(leg_wheel_odom_node
  rclcpp sensor_msgs nav_msgs geometry_msgs)
install(TARGETS leg_wheel_odom_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
```

- [ ] **Step 3: 构建**

Run: `cd /mnt/d/light && source install/setup.bash && colcon build --packages-select leg_wheel_odom`
Expected: 构建成功无警告（`-Wall` 下如有 unused 变量需清理）。

- [ ] **Step 4: 仿真冒烟验证（Gazebo M20 直行）**

Run:
```bash
cd /mnt/d/light && source install/setup.bash
ros2 run leg_wheel_odom leg_wheel_odom_node --ros-args --params-file src/leg_wheel_odom/config/leg_wheel_odom.yaml &
ros2 topic echo /odom_wheel/twist --once
```
Expected: Gazebo 直行时 `linear.x ≈ 轮速×0.05`，`angular.z ≈ 0`；原地旋转时相反。

- [ ] **Step 5: Commit**

```bash
cd /mnt/d/light && git add src/leg_wheel_odom && git commit -m "feat: leg_wheel_odom 节点（轮式/腿式状态机、打滑检测、协方差输出）"
```

---

### Task 3: lightning-lm fork —— odom 输入管线接线

**Files:**
- Modify: `lightning-lm/src/core/lio/laser_mapping.h`
- Modify: `lightning-lm/src/core/lio/laser_mapping.cc`
- Modify: `lightning-lm/src/core/system/slam.cc`、`slam.h`
- Modify: `lightning-lm/src/app/run_slam_offline.cc`
- Modify: `lightning-lm/config/default_robosense.yaml`
- Modify: `lightning-lm/src/wrapper/bag_io.h`（启用 odom 反序列化）

**Interfaces:**
- Consumes: `/odom_wheel`（Task 2，`nav_msgs/msg/Odometry`）
- Produces: `LaserMapping::ProcessOdom(const OdomPtr& odom)`（线程安全入缓冲）
- Produces: `measures_.odom_`（`std::deque<OdomPtr>`，SyncPackages 填充，含 scan 区间内的 odom）
- Produces: yaml 键 `common.odom_topic`（默认 `/odom_wheel`）

- [ ] **Step 1: laser_mapping.h 声明**

在 `laser_mapping.h` 的 `void ProcessIMU(const lightning::IMUPtr &msg_in);`（约 79 行）后加：
```cpp
    /// 处理轮速/腿式里程计（Odom），线程安全
    void ProcessOdom(const OdomPtr &odom);
```
在私有段 `std::deque<lightning::IMUPtr> imu_buffer_;`（约 189 行）后加：
```cpp
    std::mutex mtx_odom_;
    std::deque<OdomPtr> odom_buffer_;  // 轮速里程计缓存
    SE3 prev_frame_pose_;              // 上一帧状态位姿（轮速观测预测用）
    bool last_frame_degenerate_ = false;  // 上一帧是否退化（轮速增强用）
```

- [ ] **Step 2: laser_mapping.cc 实现 ProcessOdom 与 SyncPackages 扩展**

在 `laser_mapping.cc` 的 `ProcessIMU` 实现后追加：
```cpp
void LaserMapping::ProcessOdom(const OdomPtr &odom) {
    UL lock(mtx_odom_);
    if (!odom_buffer_.empty() && odom->timestamp_ < odom_buffer_.back()->timestamp_) {
        LOG(WARNING) << "odom loop back, clear buffer";
        odom_buffer_.clear();
    }
    odom_buffer_.push_back(odom);
    while (odom_buffer_.size() > 200) {
        odom_buffer_.pop_front();
    }
}
```

在 `SyncPackages()` 的 imu 同步段（`measures_.imu_.clear(); ... pop_front();` 之后、`lidar_buffer_.pop_front();` 之前）插入：
```cpp
    /*** push odom_ data, and pop from odom buffer ***/
    UL lock_odom(mtx_odom_);
    measures_.odom_.clear();
    if (!odom_buffer_.empty()) {
        while (!odom_buffer_.empty() && odom_buffer_.front()->timestamp_ < measures_.lidar_begin_time_ - 0.1) {
            odom_buffer_.pop_front();  // 丢弃过期
        }
        for (auto &odom : odom_buffer_) {
            if (odom->timestamp_ > measures_.lidar_end_time_ + 0.1) {
                break;
            }
            measures_.odom_.push_back(odom);
        }
        odom_buffer_.clear();
    }
```

- [ ] **Step 3: Run() 更新触发改选观测类型**

将 `laser_mapping.cc:267` 的 `kf_.Update(ESKF::ObsType::LIDAR, 1.0);` 替换为：
```cpp
    if (!measures_.odom_.empty()) {
        kf_.Update(ESKF::ObsType::WHEEL_SPEED_AND_LIDAR, 1.0);
    } else {
        kf_.Update(ESKF::ObsType::LIDAR, 1.0);
    }
```
在 `state_point_ = kf_.GetX(); state_point_.timestamp_ = measures_.lidar_end_time_;`（269-270 行）之后加：
```cpp
    prev_frame_pose_ = state_point_.GetPose();
```

- [ ] **Step 4: slam.cc 订阅 odom 话题**

在 `slam.cc` 的 `imu_topic_`/`cloud_topic_` 读取处（87-89 行附近）加：
```cpp
    odom_topic_ = yaml["common"]["odom_topic"].as<std::string>("/odom_wheel");
```
在 `cloud_sub_`/`livox_sub_` 创建处（106-111 行之后）加：
```cpp
    if (!odom_topic_.empty()) {
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                OdomPtr odom = std::make_shared<Odom>();
                odom->timestamp_ = ToSec(msg->header.stamp);
                odom->pose = SE3(Quatd(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
                                 Vec3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
                odom->linear = Vec3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
                odom->angular = Vec3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
                lio_->ProcessOdom(odom);
            });
    }
```
> 需要确认 `ToSec` 与 `Quatd` 在 `wrapper/ros_utils.h` 或 `common/eigen_types.h` 的拼写；编译报错时按仓库实际名称修正（`slam.cc` 内 imu 回调已有同类转换可参考）。

在 `slam.h` 私有段 `std::string imu_topic_;`（97 行）后加：
```cpp
    std::string odom_topic_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
```

- [ ] **Step 5: 离线回放支持（bag_io + run_slam_offline）**

在 `wrapper/bag_io.h` 中，`OdomHandle` 定义后（54 行附近）启用 odom 处理：取消 `seri_odom_` 注释，并新增：
```cpp
    void ProcessOdom(const rosbag2_cpp::SerializedBagMessageSharedPtr &msg) {
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        seri_odom_.deserialize_message(msg->serialized_data.get(), odom_msg.get());
        OdomPtr odom = std::make_shared<Odom>();
        odom->timestamp_ = ToSec(odom_msg->header.stamp);
        odom->pose = SE3(Quatd(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                               odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z),
                         Vec3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z));
        odom->linear = Vec3d(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);
        odom->angular = Vec3d(odom_msg->twist.twist.angular.x, odom_msg->twist.twist.angular.y, odom_msg->twist.twist.angular.z);
        if (odom_handle_) odom_handle_(odom);
    }
```
在 `run_slam_offline.cc` 中 imu/cloud handler 注册处后加：
```cpp
    bag_io.SetOdomHandle([&](const OdomPtr &odom) { slam.Lio()->ProcessOdom(odom); });
```
> 若 `SlamSystem` 未暴露 `Lio()`，在 `slam.h` 加 `std::shared_ptr<LaserMapping> Lio() { return lio_; }`；离线模式若 odom 话题不在 bag 中则该 handler 不触发，属正常。

- [ ] **Step 6: yaml 配置**

在 `config/default_robosense.yaml` 的 `common:` 段加：
```yaml
  odom_topic: /odom_wheel
```

- [ ] **Step 7: 构建 + 无 odom 回归验证**

Run:
```bash
cd /mnt/d/light && source install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 run lightning run_slam_offline --input_bag /mnt/d/data/ysc/<bag>.db3 --config ./lightning-lm/config/default_robosense.yaml
```
Expected: 构建成功；离线建图结果与 Task 0 基线一致（无 odom 话题时融合路径不触发，行为零变化）。

- [ ] **Step 8: Commit**

```bash
cd /mnt/d/light && git add lightning-lm && git commit -m "feat: lightning-lm odom 输入管线（ProcessOdom/SyncPackages/订阅/离线回放）"
```

---

### Task 4: 轮速观测函数 + ESKF gap 修复（TDD）

**Files:**
- Create: `lightning-lm/src/core/lio/wheelspeed_obs.h`
- Create: `lightning-lm/src/core/lio/wheelspeed_obs.cc`
- Modify: `lightning-lm/src/core/lio/eskf.cc`（AND 分支）
- Modify: `lightning-lm/src/core/lio/laser_mapping.h`、`laser_mapping.cc`（接线）
- Create: `lightning-lm/tests/test_wheelspeed_obs.cpp`（gtest）
- Modify: `lightning-lm/CMakeLists.txt`（测试目标）

**Interfaces:**
- Consumes: `measures_.odom_`、`prev_frame_pose_`、`ESKF::CustomObservationModel`
- Produces: 自由函数 `void BuildWheelObs(const SE3 &pred_inc, const SE3 &meas_inc, double weight, ESKF::CustomObservationModel &obs);`（纯数学，可单测）
- Produces: `LaserMapping::WheelSpeedObsModel(NavState&, ESKF::CustomObservationModel&)`

- [ ] **Step 1: 写失败测试**

Create `lightning-lm/tests/test_wheelspeed_obs.cpp`:
```cpp
#include <gtest/gtest.h>
#include "core/lio/wheelspeed_obs.h"

using namespace lightning;

TEST(WheelObs, ConstraintStructure) {
    // 纯 x 平移增量：只约束 x 与 yaw
    SE3 pred(SO3(), Vec3d(1.0, 0.0, 0.0));
    SE3 meas(SO3(), Vec3d(1.0, 0.0, 0.0));
    ESKF::CustomObservationModel obs;
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
    BuildWheelObs(pred, meas, 1.0, obs);
    EXPECT_GT(obs.HTr_(0), 0.0);
}

TEST(WheelObs, WeightScaling) {
    SE3 pred(SO3(), Vec3d(1.0, 0.0, 0.0));
    SE3 meas(SO3(), Vec3d(1.5, 0.0, 0.0));
    ESKF::CustomObservationModel a, b;
    BuildWheelObs(pred, meas, 1.0, a);
    BuildWheelObs(pred, meas, 4.0, b);
    EXPECT_NEAR(b.HTH_(0, 0), 4.0 * a.HTH_(0, 0), 1e-9);
}
```

- [ ] **Step 2: 运行确认失败**

Run: `cd /mnt/d/light && source install/setup.bash && colcon build --packages-select lightning --cmake-args -DCMAKE_BUILD_TYPE=Release`
Expected: 编译失败（找不到 `core/lio/wheelspeed_obs.h`）。

- [ ] **Step 3: 实现自由函数**

Create `lightning-lm/src/core/lio/wheelspeed_obs.h`:
```cpp
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
```

Create `lightning-lm/src/core/lio/wheelspeed_obs.cc`:
```cpp
#include "core/lio/wheelspeed_obs.h"

namespace lightning {

void BuildWheelObs(const SE3 &pred_inc, const SE3 &meas_inc, double weight, ESKF::CustomObservationModel &obs) {
    // 残差：r = meas_inc ⊖ pred_inc（SE3 切线空间，6 维）
    const Vec3d r_pos = pred_inc.rotation().transpose() * (meas_inc.translation() - pred_inc.translation());
    const Vec3d r_rot = SO3::Log(pred_inc.rotation().transpose() * meas_inc.rotation());

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
```

- [ ] **Step 4: 修 ESKF AND 分支 gap**

将 `eskf.cc` 的观测函数分发段（131-141 行）替换为：
```cpp
        if (obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR) {
            lidar_obs_func_(x_, custom_obs_model_);
        }
        if (obs == ObsType::WHEEL_SPEED || obs == ObsType::WHEEL_SPEED_AND_LIDAR) {
            wheelspeed_obs_func_(x_, custom_obs_model_);
        }
        if (obs == ObsType::ACC_AS_GRAVITY) {
            acc_as_gravity_obs_func_(x_, custom_obs_model_);
        }
        if (obs == ObsType::GPS) {
            gps_obs_func_(x_, custom_obs_model_);
        }
        if (obs == ObsType::BIAS) {
            bias_obs_func_(x_, custom_obs_model_);
        }
```

- [ ] **Step 5: LaserMapping 接线**

在 `laser_mapping.h` 的 `void ObsModel(NavState &s, ESKF::CustomObservationModel &obs);`（122 行）后加：
```cpp
    void WheelSpeedObsModel(NavState &s, ESKF::CustomObservationModel &obs);
```

在 `laser_mapping.cc` 的 `Init()` 中 `eskf_options.lidar_obs_func_ = ...`（31 行）后加：
```cpp
    eskf_options.wheelspeed_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) {
        WheelSpeedObsModel(s, obs);
    };
```

新建 `lightning-lm/src/core/lio/wheelspeed_obs_model.cc`（或追加到 wheelspeed_obs.cc 末尾）：
```cpp
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
    const SE3 meas_inc = odom_t0->pose.inverse() * odom_t1->pose;

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
```
> `wheel_odom_weight_`、`wheel_degeneracy_boost_`、`last_frame_degenerate_` 为 laser_mapping.h 需新增的成员（见 Task 5 Step 1；本 Task 可先用默认值 1.0/1.0/false 编译，Task 5 再完整接入）。

- [ ] **Step 6: CMake 加入测试目标**

在 `lightning-lm/CMakeLists.txt` 末尾 `ament_package()` 前加：
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wheelspeed_obs tests/test_wheelspeed_obs.cpp)
  target_include_directories(test_wheelspeed_obs PRIVATE src)
  target_link_libraries(test_wheelspeed_obs lightning)
endif()
```

- [ ] **Step 7: 构建 + 测试**

Run: `cd /mnt/d/light && source install/setup.bash && colcon build --packages-select lightning --cmake-args -DCMAKE_BUILD_TYPE=Release && colcon test --packages-select lightning --event-handlers console_direct+`
Expected: `test_wheelspeed_obs` 3 个用例 PASS（`ConstraintStructure`/`ResidualSign`/`WeightScaling`）。

- [ ] **Step 8: Commit**

```bash
cd /mnt/d/light && git add lightning-lm && git commit -m "feat: 轮速观测模型（BuildWheelObs）+ ESKF AND 分支 gap 修复"
```

---

### Task 5: 退化感知增强（degeneracy 暴露 + 轮速增强 + yaml）

**Files:**
- Modify: `lightning-lm/src/core/lio/eskf.hpp`、`eskf.cc`（回调）
- Modify: `lightning-lm/src/core/lio/laser_mapping.h`、`laser_mapping.cc`（成员/注册/GetDegeneracy）
- Modify: `lightning-lm/src/core/system/slam.cc`（发布 `/degeneracy_status`）
- Modify: `lightning-lm/config/default_robosense.yaml`

**Interfaces:**
- Consumes: Task 4 的 `wheel_odom_weight_`/`wheel_degeneracy_boost_` 占位
- Produces: `ESKF::Options::degeneracy_callback_`：`std::function<void(int nullity, const Vec6d &eigenvalues)>`
- Produces: `LaserMapping::GetDegeneracyInfo(int &nullity, Vec6d &eigenvalues) const`
- Produces: `/degeneracy_status`（`std_msgs/msg/Float32MultiArray`：`[nullity, 6 个特征值...]`）

- [ ] **Step 1: ESKF 回调**

在 `eskf.hpp` 的 `Options` 中 `degeneracy_cov_inflation_`（86 行）后加：
```cpp
        /// 退化回调：更新成功时上报退化方向数与特征值（可空）
        std::function<void(int nullity, const Vec6d &eigenvalues)> degeneracy_callback_;
```

在 `eskf.cc` 的 P 膨胀段（345-349 行）内加：
```cpp
            if (options_.degeneracy_callback_) {
                options_.degeneracy_callback_(nullity, eigen_values);
            }
```
（`nullity` 与 `eigen_values` 已在该作用域内，可直接使用。）

- [ ] **Step 2: LaserMapping 接线**

在 `laser_mapping.h` 的 `WheelSpeedObsModel` 声明后加：
```cpp
    /// 获取上一帧退化信息
    void GetDegeneracyInfo(int &nullity, Vec6d &eigenvalues) const {
        nullity = last_nullity_;
        eigenvalues = last_eigenvalues_;
    }
```
私有成员（`prev_frame_pose_` 附近）加：
```cpp
    int last_nullity_ = 0;
    Vec6d last_eigenvalues_ = Vec6d::Zero();
    double wheel_odom_weight_ = 1.0;   // 轮速观测权重（yaml: wheel_odom_weight）
    double wheel_degeneracy_boost_ = 1.0;  // 退化时轮速增强倍率（yaml: wheel_degeneracy_boost）
```

在 `laser_mapping.cc` 的 `Init()` 中 `kf_.Init(eskf_options);`（33 行）前加：
```cpp
    eskf_options.degeneracy_callback_ = [this](int nullity, const Vec6d &eigenvalues) {
        last_nullity_ = nullity;
        last_eigenvalues_ = eigenvalues;
        last_frame_degenerate_ = nullity > 0;
    };
```

在 `LoadParamsFromYAML` 的 fasterlio 段末尾（`options_.proj_kfs_` 读取后）加：
```cpp
        wheel_odom_weight_ = yaml["fasterlio"]["wheel_odom_weight"].as<double>(1.0);
        wheel_degeneracy_boost_ = yaml["fasterlio"]["wheel_degeneracy_boost"].as<double>(1.0);
        eskf_options_degeneracy_threshold_ratio_ = yaml["fasterlio"]["degeneracy_threshold_ratio"].as<double>(1e-3);
        eskf_options_degeneracy_cov_inflation_ = yaml["fasterlio"]["degeneracy_cov_inflation"].as<double>(1.02);
```
> 后两项需要传入 `Init()` 的 `eskf_options`，因此把 `Init()` 中 `ESKF::Options eskf_options;`（28 行）改为成员 `ESKF::Options eskf_options_;`，LoadParamsFromYAML 里直接写 `eskf_options_.degeneracy_threshold_ratio_ = ...`；Init 内改用 `eskf_options_`。

- [ ] **Step 3: /degeneracy_status 发布**

在 `slam.cc` 的成员初始化处加发布器（约 94-111 行订阅创建后）：
```cpp
        degeneracy_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/degeneracy_status", 10);
        degeneracy_timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this]() {
            int nullity = 0;
            Vec6d eigenvalues = Vec6d::Zero();
            if (lio_) lio_->GetDegeneracyInfo(nullity, eigenvalues);
            std_msgs::msg::Float32MultiArray msg;
            msg.data.push_back(nullity);
            for (int i = 0; i < 6; ++i) msg.data.push_back(eigenvalues[i]);
            degeneracy_pub_->publish(msg);
        });
```
`slam.h` 私有段加：
```cpp
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr degeneracy_pub_;
    rclcpp::TimerBase::SharedPtr degeneracy_timer_;
```
`default_robosense.yaml` 的 `fasterlio:` 段加：
```yaml
  wheel_odom_weight: 1.0
  wheel_degeneracy_boost: 4.0
  degeneracy_threshold_ratio: 0.001
  degeneracy_cov_inflation: 1.02
```

- [ ] **Step 4: 构建 + 无 odom 回归**

Run: `cd /mnt/d/light && source install/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && colcon test --packages-select lightning --event-handlers console_direct+`
Expected: 构建成功；`test_wheelspeed_obs` 仍 PASS；再跑一次 Task 0 的 ysc 离线建图，结果与基线一致。

- [ ] **Step 5: Commit**

```bash
cd /mnt/d/light && git add lightning-lm && git commit -m "feat: 退化感知增强（degeneracy 回调暴露 + 轮速退化增强 + yaml 参数）"
```

---

### Task 6: Gazebo M20 仿真验证套件

**Files:**
- Create: `sim_ws/src/m20_description/`（复用既有 M20 URDF 适配脚本产出，含 diff_drive/站立控制器、LiDAR+IMU 传感器、ring 字段补全节点）
- Create: `sim_ws/worlds/corridor.sdf`、`sim_ws/worlds/plaza.sdf`、`sim_ws/worlds/office.sdf`
- Create: `sim_ws/scripts/run_eval.sh`、`sim_ws/scripts/ring_fill_node.py`

**Interfaces:**
- Consumes: Task 1-5 全部产物
- Produces: 三场景 × 两版本（原生/改进）的轨迹文件与 evo 指标表

- [ ] **Step 1: 建 M20 仿真环境**

复用此前 M20 Gazebo 方案（URDF 补 transmission/gazebo 标签、`diff_drive_controller` + 腿站立 `joint_trajectory_controller`、ray LiDAR→PointCloud2 + IMU 200Hz），放入 `sim_ws`。启动验证：
Run: `cd /mnt/d/light/sim_ws && source install/setup.bash && ros2 launch m20_description gazebo.launch.py`
Expected: Gazebo 中 M20 站立稳定，`/joint_states` 16 关节输出、LiDAR 点云与 IMU 话题存在。

- [ ] **Step 2: ring 字段补全节点**

Create `sim_ws/scripts/ring_fill_node.py`（Gazebo gpu_lidar 点云无 ring 字段，按垂直角补线束号，16 线 -15°~15°）：
```python
#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct

class RingFillNode(Node):
    def __init__(self):
        super().__init__('ring_fill_node')
        self.lines = self.declare_parameter('lines', 16).value
        self.pub = self.create_publisher(PointCloud2, '/rs_points', 10)
        self.sub = self.create_subscription(PointCloud2, '/points_raw', self.cb, 10)
        fov = 30.0 * math.pi / 180.0
        self.angles = np.linspace(-fov/2, fov/2, self.lines)
        self.fields = list(self.build_fields())
        self.fmt = '<fffIB'

    def build_fields(self):
        yield PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1)
        yield PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1)
        yield PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        yield PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        yield PointField(name='ring', offset=16, datatype=PointField.UINT8, count=1)

    def cb(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        out = bytearray()
        for p in raw:
            ang = math.atan2(p[2], math.hypot(p[0], p[1]))
            ring = int(np.argmin(np.abs(self.angles - ang)))
            out += struct.pack(self.fmt, p[0], p[1], p[2], 0.0, ring)
        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height, out_msg.width = 1, len(raw)
        out_msg.fields = self.fields
        out_msg.point_step, out_msg.row_step = 17, 17 * len(raw)
        out_msg.is_dense = True
        out_msg.data = bytes(out)
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    rclpy.spin(RingFillNode())
    rclpy.shutdown()
```

- [ ] **Step 3: 三个测试世界**

走廊（corridor.sdf）：长 30m × 宽 2m 直墙，两端开口；广场（plaza.sdf）：60m×40m 空地四周有墙；办公（office.sdf）：5 间房 + 门洞 + 货架。SDF 世界以简单 box 组合（在此不展开全部 XML，参照 Gazebo 示例 world 语法：`<world><model><link><collision/visual><geometry><box>`）。

- [ ] **Step 4: 数据采集脚本**

Create `sim_ws/scripts/run_eval.sh`（要点，参数化场景名与版本）：
```bash
#!/bin/bash
# 用法: ./run_eval.sh corridor baseline|improved
set -e
SCENE=$1; VERSION=$2
source /mnt/d/light/install/setup.bash
ros2 launch m20_description gazebo.launch.py world:=$SCENE &
sleep 8
# 遥操作轨迹：走廊直行往返 + 中途停驻（退化重点段）
ros2 run leg_wheel_odom leg_wheel_odom_node --ros-args --params-file /mnt/d/light/src/leg_wheel_odom/config/leg_wheel_odom.yaml &
if [ "$VERSION" = "improved" ]; then
  ros2 run lightning run_slam_online --config /mnt/d/light/lightning-lm/config/default_robosense.yaml &
else
  sed 's|odom_topic: /odom_wheel|odom_topic: ""|' /mnt/d/light/lightning-lm/config/default_robosense.yaml > /tmp/no_odom.yaml
  ros2 run lightning run_slam_online --config /tmp/no_odom.yaml &
fi
ros2 bag record -o /mnt/d/data/${SCENE}_${VERSION} -a &
sleep 120  # 采集时长
kill %1 %2 %3 %4 2>/dev/null || true
```
> 真值：Gazebo 模型位姿经 `ros_gz bridge` 的 `/world/pose` 话题或 `gz model --pose` 查询，转 TUM 轨迹；LIO 轨迹从 `/tf map→base_link` 或 SLAM 的轨迹话题记录。

- [ ] **Step 5: evo 评测**

Run:
```bash
pip install evo --upgrade --break-system-packages
evo_ape tum /mnt/d/data/${SCENE}_gt.tum /mnt/d/data/${SCENE}_${VERSION}_traj.tum -va --plot --save_result /mnt/d/data/${SCENE}_${VERSION}.zip
```
Expected: 走廊场景 improved 版 ATE/RPE 显著低于 baseline（目标：走廊段漂移 < baseline 50% 或 < 0.5m/100m）；办公场景两版 ATE 差 < 10%。`/degeneracy_status` 在走廊直行段 nullity ≥ 1。

- [ ] **Step 6: Commit**

```bash
cd /mnt/d/light && git add sim_ws && git commit -m "test: Gazebo M20 仿真验证套件（三场景 + 采集/评测脚本）"
```

---

### Task 7: 数据集回归 + 性能测量 + 真机验证清单

**Files:**
- Create: `sim_ws/scripts/perf_measure.sh`
- Modify: 无（验证为主）

**Interfaces:**
- Consumes: Task 0-6 全部
- Produces: 验收报告 `docs/验收报告_LIO前端增强.md`

- [ ] **Step 1: 数据集回归（负向测试）**

Run（NCLT + ysc，均无 odom 话题）:
```bash
cd /mnt/d/light && source install/setup.bash
ros2 run lightning run_slam_offline --input_bag /mnt/d/data/nclt/<bag>.db3 --config ./lightning-lm/config/default_nclt.yaml
ros2 run lightning run_slam_offline --input_bag /mnt/d/data/ysc/<bag>.db3 --config ./lightning-lm/config/default_robosense.yaml
```
Expected: 与 Task 0 基线一致（无 odom 时零行为变化），确认融合路径不误触发。

- [ ] **Step 2: RK3588 性能测量**

在 M20 机载（或 WSL 内核限制模拟）运行在线建图 10 分钟，测：
```bash
ps -o pid,pcpu,pmem,rss,comm -p $(pgrep -f run_slam_online)   # 每 5s 采样一次
```
Create `sim_ws/scripts/perf_measure.sh`：每 5s 采样 run_slam_online + leg_wheel_odom 两进程 CPU/内存，输出 CSV 与均值。
Expected: lightning-lm 进程 CPU 增量 ≤ 0.3 核（对比基线），新增节点 ≤ 0.1 核，RSS 无持续增长（30 分钟内）。

- [ ] **Step 3: 真机 M20 验证清单**

在真机执行（记录结果入验收报告）：
1. 开机 → 启动 lightning-lm 建图 + leg_wheel_odom → 走廊直行 100m → 返回起点，量测闭合误差（目标 < 0.5m）
2. 同走廊无 odom 版对比（目标：改进版漂移更低）
3. 轮足模式切换（站起/坐下）时轨迹无跳变
4. 打滑诱发（瓷砖地急加速）下定位不丢失
5. 全程 2h 长跑，监控 `/degeneracy_status` 与内存

- [ ] **Step 4: 验收报告**

Create `docs/验收报告_LIO前端增强.md`：三场景 evo 表格、退化占比统计、性能 CSV 摘要、真机结果、遗留问题与参数记录。
Run: `cd /mnt/d/light && git add docs && git commit -m "docs: LIO 前端增强验收报告"`

---

## Self-Review 记录

- 规格覆盖：第 4 节（腿/轮节点）→ Task 1-2；第 5 节（轮速注入）→ Task 3-4；第 6 节（退化增强）→ Task 5；第 8 节（测试验证）→ Task 6-7；第 7 节（错误处理）内嵌于 Task 2 打滑检测/协方差与 Task 3 时间戳检查。无遗漏。
- 占位符：无 TBD/TODO；`<bag>.db3` 为数据集文件名占位（README 明确提供下载源，属合理参数化）。
- 类型一致性：`BuildWheelObs(pred_inc, meas_inc, weight, obs)` 在 Task 4 定义与 Task 4 Step 5 调用签名一致；`GetDegeneracyInfo(int&, Vec6d&)` 在 Task 5 定义与 Step 3 调用一致；`wheel_odom_weight_`/`wheel_degeneracy_boost_` 在 Task 4 占位声明、Task 5 完整定义，成员名一致。
- 风险提示：Task 3 Step 4/5 中 `ToSec`/`Quatd`/`SlamSystem::Lio()` 的具体拼写以仓库实际为准（已注明"编译报错时按仓库实际修正"）；M20 关节名（`fl_hipy_joint` 等）来自既有 M20 控制器配置。
