# light_slam × 《M20 Pro 原厂基线交接文档》对齐核对

> 依据：《M20 Pro 原厂基线交接文档》（2026-09-01 只读探查实录，即"对齐参考"）
> 与 `m20_orignal` 仓库的适配结构。本文档给出 light_slam 逐项对齐结论、
> 安全边界、导航对接链路与真机验收 SOP。

---

## 1. 系统基线核对（对齐参考 §1-§2）

| 项 | 原厂实录 | light_slam 侧 | 结论 |
|---|---|---|---|
| 接入主机 | **AOS** `192.168.101.36`（p2p0，Wi-Fi 客户端），eth0 `10.21.33.103`；NOS `10.21.31.106`/GOS `10.21.31.104` **无 SSH**（黑盒） | 部署目标 = AOS（`user@192.168.101.36`）；`start_slam.sh`/`start_loc.sh` 在 AOS 上运行 | ✅ 文档已修正（此前误写 GOS/NOS 可 SSH） |
| 系统/中间件 | Ubuntu 20.04 aarch64，ROS2 Foxy + rmw_fastrtps_cpp，域 0；root-ro 另有一套 Humble **勿混用** | `start_*.sh` 优先 `source /opt/robot/scripts/setup_ros2.sh`（写 fastdds.xml 白名单并 source Foxy），`ROS_DOMAIN_ID=0`，`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | ✅ |
| 板载库版本 | PCL 1.10 / Eigen 3.3.7 / OpenCV 4.2 / yaml-cpp 0.6.2 | lightning-lm 依赖 PCL/OpenCV/yaml-cpp/Eigen，`scripts/install_dep.sh` 自动识别 Foxy/Humble | ✅（无 GTSAM 等额外硬依赖） |
| 实时进程绑核 | SSH 会话限制在 A55 小核 0-3，大核 4-7 被原厂实时服务占用 | 运行预算按小核评估；性能合格线见 §8 | ⚠️ 验收项 |

## 2. drdds / DDS 层核对（对齐参考 §3）

| 项 | 原厂实录 | light_slam 侧 | 结论 |
|---|---|---|---|
| 传输 | FastDDS 2.14，域 0，类型名 ROS2 风格；ROS2(Foxy) 节点与原生 drdds 节点域 0 **直接互通，零桥接** | `m20_joints_adapter` 直接用 ROS2 `drdds::msg::JointsData`（rosidl 生成包）订阅 `/JOINTS_DATA`；点云/IMU 用标准 ROS2 消息订阅 `/LIDAR/POINTS`、`/IMU` | ✅ 符合 §3.2 互通机制 |
| 消息包 | `/opt/ros/foxy/share/drdds`（rosidl 生成）；原生 SDK `/usr/local/include/drdds` | 只用 ROS2 消息包（**不加载**厂商原生 `libdrdds.so`，从源头规避同进程双 FastDDS ABI 冲突，m20_orignal 用独立 receiver 解决的正是此问题） | ✅ |
| fastdds.xml 白名单 | 仅 127.0.0.1 + 本机 eth0 IP，外部设备跨网 DDS 默认不通 | 节点全部在 AOS 板上运行（板载接入），不依赖跨网 | ✅ |
| QoS | drqos.xml：点云 RELIABLE+deadline 50ms；`/IMU` 实测有 RELIABLE publisher | `system.sensor_best_effort=true`（best_effort 订阅兼容 reliable/best_effort 发布端）；`topic_ready_check.py` 亦用 best_effort | ✅ |

## 3. 话题与消息契约核对（对齐参考 §5）

### 建图/定位输入

| 话题 | 原厂类型 | light_slam 订阅 | 结论 |
|---|---|---|---|
| `/LIDAR/POINTS` | `sensor_msgs/PointCloud2`（96 线融合云，fields x/y/z/intensity/ring/timestamp double） | ✅ `common.lidar_topic` | ✅ |
| `/IMU`（或 `/IMU_YESENSE`） | `sensor_msgs/Imu`，200Hz | ✅ `common.imu_topic`（默认 `/IMU`；原厂 LIO 用 `/IMU_YESENSE`，可按真机实测切换） | ✅ |
| `/JOINTS_DATA` | `drdds/msg/JointsData`（16 关节） | ✅ `m20_joints_adapter` → `/joint_states` | ✅ |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | ✅ 定位节点订阅（§6.4 重定位） | ✅ |

### 建图/定位输出

| 话题 | light_slam 发布 | 与原厂关系 | 结论 |
|---|---|---|---|
| `/lio_pose`（`system.lio_pose_topic`，默认 `/lio_pose`） | `geometry_msgs/PoseStamped`，10Hz | 独立话题（原厂 LIO 输出 `/LIO_ODOM` 等，不冲突） | ✅ 安全边界 |
| `/ODOM`（`system.odom_topic`，默认 `/ODOM`） | `nav_msgs/Odometry`，map 系，10Hz | 原厂 NOS localization 也发布 `/ODOM`（1 pub 4 sub）。**接管语义**：部署时须确认原厂定位未启用，或改 `odom_topic` 为隔离话题 | ⚠️ 见 §4 |
| `map -> base_link` TF（`system.pub_tf`） | `tf2` | 原厂 NOS 定位亦发布 TF。**接管语义**同上 | ⚠️ 见 §4 |
| `occ_grid.pgm/yaml` | 2.5D 栅格（`g2p5.grid_map_resolution`） | 对齐 drmap 激活目录 `/var/opt/robot/data/maps/active/` | ✅ §5 |
| `/lightning/save_map` | `lightning/srv/SaveMap` 服务 | 独立服务名 | ✅ |

### 导航契约（对齐参考 §5.1 规划/控制区）

| 话题/服务 | 原厂类型 | light_slam 是否涉及 | 结论 |
|---|---|---|---|
| `/GRID_MAP` | `nav_msgs/OccupancyGrid` | 不消费（原厂 NOS 发布） | ✅ 只读 |
| `/GOAL_GLOBAL`、`/GOAL_PLANNER` | `drdds/srv/PoseStampedToInt32`（命令口） | 不消费（原厂 App/basic_server 转发） | ✅ 只读 |
| `/NAV_CMD` | `drdds/msg/NavCmd`（x_vel/y_vel/yaw_vel） | **不写**（light_slam 不发任何运动指令） | ✅ 安全边界 |
| `/PLANNER_STATUS`、`/GLOBAL_PLANNER_STATUS` | `drdds/msg/PlannerStatus` | 不涉及 | ✅ |

## 4. 安全边界（对齐参考 §10-§11）

1. **不写入 `/NAV_CMD`、不接管运动链**：light_slam 只做建图/定位/里程计融合，
   不发运动指令；`start_*.sh` 无任何 `cmd_vel`/`NavCmd` 输出路径。✅
2. **`/ODOM` 与 TF 解耦**（本轮代码补强）：`/ODOM` 发布独立于 `pub_tf`；
   `system.pub_tf=false` 可关闭 `map->base_link` TF 而 `/ODOM` 照常输出，
   避免与原厂定位链 TF 冲突。✅
3. **话题名可隔离**：`system.odom_topic`（默认 `/ODOM`）、`system.lio_pose_topic`
   （默认 `/lio_pose`）可改为 `/m20_slam/odom`、`/m20_slam/pose`，与原厂链路并行
   隔离验收（对齐 m20_orignal 的隔离输出思路）。✅
4. **不改原厂配置/服务**：全部节点在用户空间运行，不写 `/opt/robot`；
   `map_path` 指向 drmap `active` 目录仅为落盘位置。✅
5. **原厂定位未启用时再发布 `/ODOM`**：真机部署前先用 `ros2 topic info /ODOM`
   确认发布者数；若 NOS 定位在跑，改隔离话题并走 `--takeover` 流程（§7）。⚠️

## 5. 导航对接链路（occ_grid → drmap → 原厂 planner）

light_slam 采用"第三方只做建图/定位，导航用原厂 NOS planner"路线（对齐参考
§11 推荐路径）：

```text
[建图]  /LIDAR/POINTS + /IMU + /JOINTS_DATA
   -> lightning-lm LIO + 回环 + g2p5
   -> data/<map_id>/: full_cloud.pcd + occ_grid.pgm/yaml（resolution 对齐 drmap）

[定位]  载入地图 -> NDT-OMP + PGO -> /ODOM(map, 10Hz) [+ TF]
   -> 原厂 NOS planner 订阅 /ODOM
   -> App 下发 /GOAL_GLOBAL -> 原厂全局/局部规划 -> /NAV_CMD -> rl_deploy 运动执行
```

要点：
- `occ_grid.pgm/yaml` 已对齐 drmap 激活目录（`system.map_path`）；
  用 `drmap unpack/apply` 或手动放置到 `/var/opt/robot/data/maps/`。
- 原厂 planner 的输入契约见 `config/native/native_global_topics.yaml` 快照。

## 6. 已知阻塞项（对齐参考 §10 实测状态）

1. **`/LIDAR/POINTS` 当前未发布**（驱动在跑但雷达疑似未上电）：依赖点云的话题
   全部静默。真机部署前必须先恢复雷达上电与驱动输出。
2. **IMU 链路静默**：`/IMU_YESENSE` 发布者数 0；`/IMU` 有 publisher 但采样无消息。
3. **`/ODOM` 双发布风险**：原厂 NOS localization 与 light_slam 定位若同时发布
   `/ODOM` 会冲突（§4 处理）。

以上均为机器人侧状态/部署时序问题，非代码缺陷。

## 7. 部署 SOP（AOS）

```bash
# 0) 前置：雷达上电，/LIDAR/POINTS 与 /IMU 有样本；确认 /ODOM 发布者
ssh user@192.168.101.36
source /opt/robot/scripts/setup_ros2.sh     # Foxy + fastdds.xml 白名单
export ROS_DOMAIN_ID=0

# 1) 同步源码（排除构建产物与数据）
rsync -a --exclude .git --exclude build --exclude install --exclude log \
  --exclude data <开发机>/light_slam/ user@192.168.101.36:/home/user/light_slam/

# 2) 构建（AOS/Foxy；开发机无 drdds 时 joints_adapter 自动跳过）
cd /home/user/light_slam
colcon build --packages-select lightning leg_wheel_odom m20_joints_adapter

# 3) 建图
./scripts/start_slam.sh --map-id site_a
# 4) 定位 + 原厂导航对接
./scripts/start_loc.sh
```

## 8. 真机验收 SOP（建议指标）

| 项 | 检查方式 | 合格线 |
|---|---|---|
| 点云/IMU 输入 | `ros2 topic hz /LIDAR/POINTS /IMU` | 10Hz / 200Hz |
| 建图 CPU/内存 | `top` / `perf_sample.sh` | ≤2 核、≤1.5GB（小核预算） |
| `/ODOM` | `ros2 topic hz /ODOM` | 10Hz，map 系 |
| 定位精度 | 往返轨迹与真值对比 | 与仿真基线（APE ≤0.3m）同量级 |
| 端到端导航 | App 下发目标，原厂 planner 消费 `/ODOM` 走通 | 到达目标/避障正常 |
| 长稳 | ≥2h 运行 | RSS 增长 <10% |

## 9. 与 m20_orignal 的结构对照

| m20_orignal | light_slam（本次补全后） |
|---|---|
| `src/m20_slam_navigation/`（自研 SLAM+导航） | `lightning-lm/`（开源 SLAM 核心）+ `src/`（adapter/轮速） |
| `config/native_*.yaml` 原厂参数快照 | `lightning-lm/config/native/`（本次新增，含 README） |
| `scripts/start_mapping.sh` 一键建图 | `scripts/start_slam.sh` / `start_loc.sh`（本次新增） |
| 自研导航合同（Hybrid A*+DWA+/NAV_CMD） | **不移植**（选定"只对接原厂导航"路线） |
| 隔离输出 `/m20_slam/*` + takeover 开关 | `system.odom_topic`/`lio_pose_topic` 隔离 + `pub_tf` 开关（本次补强） |
| `docs/M20PRO_*_ADAPTATION.md` | `docs/M20_ALIGNMENT.md`（本文档）+ 既有 `M20_DEPLOYMENT.md` |

---

*本核对基于 2026-09-01 原厂基线实录；真机状态（雷达上电、IMU 链路、/ODOM 发布者）*
*以部署前现场核验为准。*
