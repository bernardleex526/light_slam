# M20 真机部署指南（lightning-lm + leg_wheel_odom）

本文档记录 D:\light 工作区针对云深处 M20 Pro 机器狗（ROS2 Foxy / RK3588 ARM）的适配修复内容与部署步骤。

## 修复总览

### P0 阻断性问题（已全部修复）

| # | 问题 | 修复 |
|---|---|---|
| 1 | `/joint_states` 话题真机不存在，leg_wheel_odom 收不到数据 | 新增 `src/m20_joints_adapter` 包，将 `/JOINTS_DATA` (drdds/msg/JointsData) 桥接为 `/joint_states` (sensor_msgs/JointState)。关节索引 0-15 → 名称映射见包内 README |
| 2 | 定位不发布 `/ODOM`，M20 planner 无法工作 | `loc_system.cc` 增加 `/ODOM` 发布器（map 系 nav_msgs/Odometry，与 TF 同源，10Hz） |
| 3 | `use_sim_time` 硬编码 true，真机时间戳失效 | `slam.cc`/`loc_system.cc` 改为从 yaml `system.use_sim_time` 读取（默认 false；仿真配置置 true） |
| 4 | x86 SSE 编译标志在 ARM(RK3588) 上编译失败 | `cmake/packages.cmake` 用 `CMAKE_SYSTEM_PROCESSOR` 守卫 SSE 标志，仅 x86 启用 |
| 5 | `scrubber_common`/`agibot_robot` 幽灵依赖 | 从 `package.xml` 删除（源码零引用） |
| 6 | 依赖脚本/Dockerfile 仅支持 Humble | `scripts/install_dep.sh` 自动识别 Foxy/Humble；新增 `docker/Dockerfile.foxy` |

### P1 重要问题（已全部修复）

| # | 问题 | 修复 |
|---|---|---|
| 7 | 轮半径 0.091 是仿真标定值 | `leg_wheel_odom.yaml` 改为 0.09（真机足轮直径 0.18m，手册 §10.4） |
| 8 | 腿长 0.28 与官方 0.25 不符 | `types.hpp` + `leg_wheel_odom.yaml` 大腿/小腿改为 0.25（手册 §10.4 L2/L4） |
| 9 | IMU 话题 `/imu` 与真机 `/IMU` 不符 | `leg_wheel_odom.yaml` 改为 `/IMU` |
| 10 | 加速度二次积分打滑检测有缺陷（无重力补偿、50Hz 系数、漂移） | 改为 IMU gyro 偏航角速度 vs 轮式差速偏航角速度一致性检测，新增 `slip_yaw_threshold` 参数（0.5 rad/s） |
| 11 | 地图输出 `map.pgm`/`map.yaml` 与 drmap 不兼容 | 改为 `occ_grid.pgm`/`occ_grid.yaml`，resolution 从 `g2p5.grid_map_resolution` 读取（不再硬编码 0.05） |
| 12 | 无法通过 `/initialpose` 重定位 | `loc_system.cc` 新增 `/initialpose` 订阅，回调 `SetInitPose` |
| 13 | 真机配置需关 UI | 新增 `config/default_m20.yaml`（with_ui/with_2dui=false, scan_line=96, sensor_best_effort=true, map_path 指向 drmap active 目录） |

### P2 次要问题（已修复）

- `localization.cpp:132` 日志复制粘贴 bug（"Using OUST 64" → "Using RoboSense"）
- 节点名冲突：定位节点改为 `lightning_loc`（建图保持 `lightning_slam`）
- QoS 可配置：`system.sensor_best_effort`（真机建议 true）
- `default_robosense.yaml` scan_line 注释澄清（NCLT=32，M20 用 default_m20.yaml）
- `default_robosense_sim.yaml` 补充 use_sim_time: true

## 真机部署步骤

### 1. 环境准备（GOS 10.21.31.104 或 NOS 10.21.31.106）

```bash
# ROS2 Foxy 环境
source /opt/robot/scripts/setup_ros2.sh
export ROS_DOMAIN_ID=0

# 安装依赖（自动识别 foxy）
cd /path/to/lightning-lm
bash scripts/install_dep.sh
# Pangolin 需手动编译安装（参考 docker/Dockerfile.foxy）
```

### 2. 构建

```bash
cd /path/to/light
# 真机上需先提供 drdds 消息包（/opt/robot 已含；若无则从 sdk_deploy 构建）
# 如需用 rosdep 安装依赖：drdds 是 M20 私有消息包、不在 rosdep 索引中，必须显式跳过：
#   rosdep install --from-paths src --ignore-src -y --skip-keys drdds
colcon build --packages-select lightning leg_wheel_odom m20_joints_adapter
source install/setup.bash
```

> 开发机（无 drdds）上 `m20_joints_adapter` 会被优雅跳过（CMake QUIET），不影响其他包构建。

### 3. 运行链路（建图）

```bash
# 1. 关节数据桥接（真机必需）
ros2 run m20_joints_adapter joints_adapter_node

# 2. 轮腿里程计
ros2 run leg_wheel_odom leg_wheel_odom_node

# 3. lightning 建图（默认使用 default_m20.yaml）
ros2 run lightning run_slam_online --config ./config/default_m20.yaml
# 保存地图
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: mymap}"
```

### 4. 运行链路（定位 + 导航）

```bash
# 1+2 同上（adapter + leg_wheel_odom）

# 3. lightning 定位：发布 /ODOM + map→base_link TF，订阅 /initialpose
ros2 run lightning run_loc_online --config ./config/default_m20.yaml

# 4. 重定位：RViz 2D Pose Estimate 或
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{...}"

# 5. 将建好的地图用于官方导航：
#    - 地图已存为 occ_grid.pgm + occ_grid.yaml
#    - 用 drmap unpack / 手动放入 /var/opt/robot/data/maps/ 后 drmap apply
#    - 官方 planner 订阅 /ODOM（现已发布）
```

## 验证清单

```bash
ros2 topic echo /joint_states          # 16 关节数据（adapter 工作）
ros2 topic echo /odom_wheel            # 轮腿里程计 50Hz
ros2 topic echo /ODOM                  # 定位位姿 10Hz（map 系）
ros2 topic hz /IMU                     # 200Hz
ros2 topic echo /initialpose           # 重定位接口
ros2 topic echo /lio_pose              # LIO 位姿（建图时）
```

## 已知遗留（需真机标定/验证）

1. **RoboSense 点云 timestamp 字段**：`PointRobotSense` 需要点云含 `timestamp` 字段（double, 秒）。真机需验证 `/LIDAR/POINTS` 字段名（`ros2 topic echo /LIDAR/POINTS --once` 检查 fields）。
   > ✅ **已用官方 M20 数据集验证（2026-08-12）**：`DeepRoboticsLab/lightning-lm-deep-robotics` office bag（`data/m20_office_bag/`）实测 `/LIDAR/POINTS` fields = `x,y,z,intensity,ring(uint16),timestamp(double @offset 18)`，frame_id=`lidar_link`，10Hz；`/IMU` 183Hz。**timestamp 字段存在且布局正确**，无阻塞。注意 header.stamp 与记录时刻有 ~124ms 固定偏移（记录管线开销，FAST-LIO2 用 header.stamp + 点内 timestamp 即可）。
2. **腿式里程计模型简化**：仅用 FL 腿 2-DOF 平面模型，忽略 hipx 与多腿融合。协方差已放大 100 倍降权，真机腿式模式下建议以 LIO 为主。（计划已决策：本次不改进，真机数据到位后按需）
3. **track_width 0.137**：取手册 W_hip，实际轮距建议真机测量校准。
4. **IMU 外参**：外参为 identity（驱动已变换到 base_link），如定位精度不满足可微调 `extrinsic_T/R`。注意：数据集 frame_id=`lidar_link`（非 base_link），离线验证时观察外参假设是否成立。
