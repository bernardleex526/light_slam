# M20 SLAM + 自研导航系统（lightning-lm + leg_wheel_odom + m20_navigation）

本仓库是面向云深处 **M20 Pro** 轮足机器狗（ROS2 Foxy / RK3588 ARM）的**完整激光-惯导建图、定位与自研导航系统**。
基于开源 [lightning-lm](https://github.com/gaoxiang12/lightning-lm)（Lightning-Speed Lidar SLAM），
叠加轮腿里程计融合、关节数据桥接，并移植了自研导航栈（Hybrid A\* + DWA + LinePlanner + 原生 DrDDS 桥），
实现从传感器输入到运动指令输出的端到端闭环。

> **三板架构**（2026-09-01 M20 Pro 原厂基线探查实录，详见 `docs/M20_ALIGNMENT.md`）：
> - **AOS** `192.168.101.36`（可 SSH）—— 第三方节点部署板
> - **NOS** `10.21.31.106`（黑盒，无 SSH）—— 原厂定位/规划
> - **GOS** `10.21.31.104`（黑盒，无 SSH）—— 原厂运动执行

---

## 1. 系统能力总览

| 能力 | 模块 | 说明 |
|---|---|---|
| **前端 LIO** | `lightning-lm` FASTER-LIO 风格 | 12 维 ESKF + ivox3d 增量体素地图，点面 ICP + 点对点 ICP（可选），支持 Livox / Velodyne / Ouster / RoboSense |
| **轮速/腿式里程计融合** | `src/leg_wheel_odom` | `/odom_wheel` 50Hz，ESKF 帧间 SE3 增量约束 x/y/yaw，退化场景自动加大轮速权重 |
| **关节桥接** | `src/m20_joints_adapter` | `/JOINTS_DATA`(drdds) → `/joint_states`（16 关节） |
| **后端回环** | `lightning-lm` grid-NDT + miao 图优化 | 自研轻量图优化库（g2o 风格，GN/LM/DogLeg） |
| **地图** | `lightning-lm` 分块点云 + g2p5 栅格 | 静态/动态层，实时输出 `occ_grid.pgm/yaml`（兼容 M20 官方 drmap） |
| **定位** | `lightning-lm` NDT-OMP + PGO | `/ODOM`（map 系，10Hz）+ `map→base_link` TF + `/initialpose` 重定位 |
| **自研导航** | `src/m20_navigation/` | **全局规划**（Hybrid A\* + 运动原语 + 样条平滑）+ **局部控制**（DWA / LinePlanner）+ **地形通行分析**（2.5D 高程格 + 坡度/粗糙度/台阶）+ **原生 DrDDS 桥**（`/NAV_CMD`、`/GOAL_GLOBAL` 等） |
| **原厂导航对接** | `lightning-lm` 定位输出 | 输出 `/ODOM` + occ_grid 给原厂 NOS planner 使用 |
| **一键启动** | `scripts/` | `start_slam.sh` / `start_loc.sh` / `start_nav.sh`（环境加载 + 预检 + Ctrl+C 自动存图/清理） |
| **仿真验证** | `sim_ws/` | M20 Gazebo 模型 + corridor/plaza/office 三场景 + 评测脚本 |

---

## 2. 目录结构

| 路径 | 说明 |
|---|---|
| `lightning-lm/` | 核心 SLAM 包：LIO 前端、回环检测、g2p5 栅格、分块地图、miao 图优化、系统接口 |
| `lightning-lm/launch/` | `m20_slam.launch.py`、`m20_loc.launch.py`、`topic_ready_check.py` 就绪门禁 |
| `lightning-lm/config/default_m20.yaml` | M20 真机默认配置（96 线 RoboSense、关 UI、QoS 等） |
| `lightning-lm/config/native/` | M20 原厂参数快照（只读参考） |
| `lightning-lm/docker/Dockerfile.foxy` | Foxy 镜像定义 |
| `src/m20_joints_adapter/` | `/JOINTS_DATA`(drdds) → `/joint_states` 桥接 |
| `src/leg_wheel_odom/` | 轮腿里程计节点（`/odom_wheel`，50Hz） |
| **`src/m20_navigation/`** | **自研导航栈**：Hybrid A\* 全局 + DWA/LinePlanner 局部 + 地形通行 + 原生 DrDDS 桥 |
| `scripts/` | `start_slam.sh` / `start_loc.sh` / `start_nav.sh` 一键启动 |
| `sim_ws/` | Gazebo 仿真验证套件 |
| `docs/` | `M20_ALIGNMENT.md`（对齐参考核对）、`M20_DEPLOYMENT.md`（真机部署）、`M20_SYSTEM_PLAN.md`（方案决策） |
| `data/` | 数据/地图目录（**不入库**） |

---

## 3. 传感器与话题约定

### 建图 / 定位输入

| 话题 | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/LIDAR/POINTS` | 输入 | `sensor_msgs/PointCloud2` | RoboSense 96 线融合云，10Hz，含 `timestamp(double)` |
| `/IMU`（或 `/IMU_YESENSE`） | 输入 | `sensor_msgs/Imu` | 200Hz；默认 `/IMU`，原厂 LIO 用 `/IMU_YESENSE` |
| `/JOINTS_DATA` | 输入 | `drdds/msg/JointsData` | 真机 16 关节数据，由 adapter 桥接 |
| `/odom_wheel` | 输入 | `nav_msgs/Odometry` | leg_wheel_odom 输出，50Hz |
| `/initialpose` | 输入 | `geometry_msgs/PoseWithCovarianceStamped` | 重定位接口（RViz 2D Pose Estimate） |

### 建图 / 定位输出

| 话题 | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/lio_pose`（`system.lio_pose_topic`） | 输出 | `geometry_msgs/PoseStamped` | 建图 LIO 位姿，10Hz；可隔离为 `/m20_slam/pose` |
| `/ODOM`（`system.odom_topic`） | 输出 | `nav_msgs/Odometry` | 定位位姿（map 系），10Hz；可隔离为 `/m20_slam/odom` |
| `map→base_link` TF（`system.pub_tf`） | 输出 | `tf2` | 定位 TF；可关闭（不影响 `/ODOM`） |
| `/lightning/save_map` | 服务 | `lightning/srv/SaveMap` | 保存地图（`map_id` 参数） |

### 导航输入（自研导航栈）

| 话题 | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/GRID_MAP` | 输入 | `nav_msgs/OccupancyGrid` | 已知静态地图；由 occ_grid_bridge 从 lightning-lm 栅格发布 |
| `/ODOM` | 输入 | `nav_msgs/Odometry` | 定位位姿 + body 系 twist（`linear.x/y`、`angular.z`） |
| `/goal_pose` | 输入 | `geometry_msgs/PoseStamped` | 全局目标点（world 系） |
| `/GOAL_GLOBAL` | 服务 | `drdds/srv/PoseStampedToInt32` | 原生 DrDDS 版目标服务（真机可选） |
| `/MOTION_INFO` | 输入 | `drdds/msg/MotionInfo` | 原厂速度反馈（真机 DrDDS 桥） |
| `/NAV_POINTS` | 输入 | `sensor_msgs/PointCloud2` | 局部点云（fallback 地形模式） |

### 导航输出（自研导航栈）

| 话题 | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/NAV_CMD` | 输出 | `geometry_msgs/Twist`（或 `drdds/msg/NavCmd`） | body 系速度命令（`linear.x=vx`、`linear.y=vy`、`angular.z=omega`）；`enable_motion_output=false` 时不发 |
| `/path_Astar` | 输出 | `nav_msgs/Path` | Hybrid A\* 全局规划路径 |
| `/global_path` | 输出 | `nav_msgs/Path` | 平滑后的全局路径 |
| `/local_goal` | 输出 | `geometry_msgs/PoseStamped` | 局部前视目标点 |
| `/PLANNER_STATUS` | 输出 | `drdds/msg/PlannerStatus` | 局部规划状态 |
| `/GLOBAL_PLANNER_STATUS` | 输出 | `drdds/msg/PlannerStatus` | 全局规划状态 |
| `/local_map` | 输出 | `nav_msgs/OccupancyGrid` | 局部代价地图 |
| `/free_paths` | 输出 | `sensor_msgs/PointCloud2` | DWA 采样轨迹 |

---

## 4. 环境与依赖

真机：**Ubuntu 20.04 + ROS2 Foxy + RK3588（AOS `192.168.101.36`，可 SSH）**。
开发机：Ubuntu 20.04/22.04 + ROS2 Foxy/Humble。

### 板载依赖

PCL 1.10、Eigen 3.3.7、OpenCV 4.2、yaml-cpp 0.6.2。
自研导航栈仅依赖 PCL/Eigen 及 rclcpp 系列标准包；另含可选厂商 DrDDS SDK（仅真机存在，CMake 通过 find_package(drdds QUIET) 自动探测）。

### 安装

```bash
cd /path/to/lightning-lm
bash scripts/install_dep.sh   # 自动识别 Foxy/Humble，apt 安装 PCL/OpenCV/yaml-cpp/glog 等
# Pangolin 需手动编译安装（见 docker/Dockerfile.foxy）
```

rosdep：

```bash
rosdep install --from-paths . src --ignore-src -y --skip-keys drdds
```

---

## 5. 构建 SOP

### 5.1 开发机（Humble）

```bash
cd /path/to/light
source /opt/ros/humble/setup.bash
colcon build --packages-select lightning leg_wheel_odom m20_joints_adapter m20_navigation
source install/setup.bash
```

开发机无 `drdds` 时，`m20_joints_adapter` 与 `m20_navigation` 的 DrDDS 桥被 CMake `QUIET` 优雅跳过。

### 5.2 真机（Foxy / RK3588 / AOS）

```bash
rsync -a --exclude .git --exclude build --exclude install --exclude log \
  --exclude data <开发机>/light_slam/ user@192.168.101.36:/home/user/light_slam/

ssh user@192.168.101.36
source /opt/robot/scripts/setup_ros2.sh   # 提供 drdds + fastdds.xml 白名单
export ROS_DOMAIN_ID=0
cd /home/user/light_slam
colcon build --packages-select lightning leg_wheel_odom m20_joints_adapter m20_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> 工作区路径含中文时 colcon 会因 rosidl UTF-8 bug 失败，请在 ASCII 路径构建。

---

## 6. 数据集准备（离线验证）

云深处官方 M20 office 数据包（推荐，与 `default_m20.yaml` 对齐）：

```bash
# 从 DeepRoboticsLab/lightning-lm-deep-robotics 下载，放到 data/m20_office_bag/
```

包含：
- `/LIDAR/POINTS`：10Hz PointCloud2，fields `x,y,z,intensity,ring,timestamp(double)`
- `/IMU`：183Hz

其余公开数据集（NCLT / VBR / UrbanLoco 等）见 `lightning-lm/README_CN.md`。

---

## 7. 离线验证 SOP

```bash
source install/setup.bash
cd /path/to/lightning-lm

# 离线建图
ros2 run lightning run_slam_offline \
  --input_bag ../data/m20_office_bag --config ./config/default_m20.yaml

# 离线定位
ros2 run lightning run_loc_offline \
  --input_bag ../data/m20_office_bag --config ./config/default_m20.yaml
```

官方 office bag 基准：建图 448 关键帧 / 49,252 点；定位 162/163 帧匹配成功，confidence 均值 2.84。

---

## 8. 在线建图 SOP（真机 AOS）

### 8.1 一键启动（推荐）

```bash
./scripts/start_slam.sh --map-id site_a
# 等价参数：--config <path> --no-joints --no-leg-odom --domain 0 --skip-preflight
```

或直接 launch：

```bash
source install/setup.bash
ros2 launch lightning m20_slam.launch.py
# 开发机无 drdds / 不接轮速时：
# ros2 launch lightning m20_slam.launch.py enable_joints_adapter:=false enable_leg_odom:=false
```

launch 顺序：joints_adapter（可选）→ leg_wheel_odom（可选）→ run_slam_online → 就绪门禁。
就绪门禁收到 `/IMU`、`/LIDAR/POINTS`、`/lio_pose`（及已启用组件的话题）后放行；60s 超时则关闭整条链路。

### 8.2 手动三节点

```bash
ros2 run m20_joints_adapter joints_adapter_node &
ros2 run leg_wheel_odom leg_wheel_odom_node &
ros2 run lightning run_slam_online --config ./config/default_m20.yaml
```

### 8.3 保存地图

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: mymap}"
# 产物：data/mymap/ 下 global.pcd（显示用）+ 分块点云 + occ_grid.pgm + occ_grid.yaml
```

---

## 9. 在线定位 SOP（真机）

AOS 板上用 `scripts/start_loc.sh`：

```bash
./scripts/start_loc.sh
# 等价参数：--config <path> --no-joints --no-leg-odom --domain 0 --skip-preflight
```

或直接 launch：

```bash
source install/setup.bash
ros2 launch lightning m20_loc.launch.py
# 或手动：
ros2 run m20_joints_adapter joints_adapter_node &
ros2 run leg_wheel_odom leg_wheel_odom_node &
ros2 run lightning run_loc_online --config ./config/default_m20.yaml
```

重定位（建图起点附近放置，或 RViz 2D Pose Estimate）：

```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

验证清单：

```bash
ros2 topic echo /joint_states   # 16 关节
ros2 topic echo /odom_wheel     # 50Hz
ros2 topic echo /ODOM           # 定位位姿 10Hz
ros2 topic hz /IMU              # 200Hz
ros2 topic echo /lio_pose       # 建图时
```

> 定位链路的就绪门禁以 `/ODOM` 为标志，需要地图已加载且 NDT 匹配成功后才会出现；
> 若定位长时间未初始化，门禁 60s 超时会关闭链路，此时请检查初始位姿与地图路径。

---

## 10. 自研导航 SOP（`src/m20_navigation`）

> 前置：已运行建图（§8）或定位（§9）链路，获得 `/ODOM` + occ_grid（或在线 `/GRID_MAP`）。

### 10.1 一键启动（推荐）

```bash
# 仅规划干跑（不发运动指令，默认安全）
./scripts/start_nav.sh

# 启用运动输出（需确认 DrDDS 链路可用）
./scripts/start_nav.sh --motion

# 使用建图产物 occ_grid.pgm 作为已知地图（发布 /GRID_MAP）
./scripts/start_nav.sh --occ-grid /var/opt/robot/data/maps/active/occ_grid.pgm

# 组合使用 + 覆盖配置
./scripts/start_nav.sh --motion --occ-grid ./data/site_a/occ_grid.pgm --config ./src/m20_navigation/config/native_navigation.yaml
```

### 10.2 直接 launch

```bash
source install/setup.bash
ros2 launch m20_navigation navigation.launch.py enable_motion_output:=false
# 或：
ros2 launch m20_navigation navigation.launch.py \
  enable_motion_output:=true enable_occ_grid_bridge:=true \
  grid_path:=/var/opt/robot/data/maps/active/occ_grid.pgm
```

launch 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `config` | `native_navigation.yaml` | navigation_node 配置（含 native 原厂参数快照） |
| `enable_motion_output` | `false` | 是否发布 `/NAV_CMD`（默认安全干跑；需确认 DrDDS 链路） |
| `enable_occ_grid_bridge` | `false` | 是否启动 occ_grid_bridge 把 lightning-lm 栅格发布为 `/GRID_MAP` |
| `grid_path` | `""` | `occ_grid.pgm` 路径（enable_occ_grid_bridge=true 时必填） |
| `log_level` | `info` | 日志级别 |

### 10.3 下发目标与可视化

```bash
# 下发全局目标（world 系）
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# RViz 查看
rviz2 -d src/m20_navigation/rviz/navigation_view.rviz
```

关键输出：`/path_Astar`、`/global_path`（Hybrid A\* + 样条平滑）、`/local_goal`、
`/local_map`、`/free_paths`（DWA 采样轨迹）、`/NAV_CMD`（body 系 vx/vy/omega）。

### 10.4 导航管线说明

- **全局规划**：Hybrid A\*（运动原语：8 方向 × 3 转向 + 3 原地旋转；Dijkstra 启发；样条平滑带碰撞回退）→ `/global_path`。
- **局部控制**：DWA（全向速度窗口、矩形机身扫掠、多代价）或 LinePlanner（默认 `directLine_mode`）→ `/NAV_CMD`（body 系）。
- **地形通行**（fallback，无 `/GRID_MAP` 时）：2.5D 高程格 + 坡度/粗糙度/台阶分析 → 通行代价地图。
- **原生 DrDDS 桥**（真机）：`/GOAL_GLOBAL` 服务、`/NAV_CMD`、`/PLANNER_STATUS`、`/GLOBAL_PLANNER_STATUS`、`/MOTION_INFO`。
- **occ_grid_bridge**：把 lightning-lm 的 `occ_grid.pgm/yaml` 读取并发布为 `/GRID_MAP`（0.5s 轮询，文件就绪前自动重试）。

> **安全边界**：默认 `enable_motion_output=false`，导航只做规划与可视化、不发出运动指令；
> 真机启用 `--motion` 前务必确认 DrDDS 链路、footprint 与 `native_navigation.yaml` 参数。

---

## 11. 仿真验证 SOP（开发机）

```bash
source /opt/ros/humble/setup.bash
source /path/to/light/sim_ws/install/setup.bash
source /path/to/light/install/setup.bash

# 11.1 启动仿真（headless，world = corridor/plaza/office）
ros2 launch m20_description gazebo.launch.py world:=corridor

# 11.2 数据采集 + 评测（baseline=无轮速融合，improved=轮速融合）
cd /path/to/light/sim_ws/scripts
./run_eval.sh corridor improved 80
./evo_eval.sh corridor improved
```

关键话题：`/joint_states`（4 轮）、`/imu`（200Hz）、`/points_raw`→`/rs_points`（ring 补全）、
`/odom_wheel`、`/lio_pose`、`/model_states`（Gazebo 真值，best_effort QoS）。

corridor 80s 往返基准：improved APE 0.23m（重跑收敛至 mean 0.165m / max 0.311m），
baseline 1.71m——轮速融合将走廊漂移降到约 13%。

---

## 12. launch 参数与就绪门禁

### 12.1 SLAM / 定位 launch（lightning）

| 参数 | 默认 | 说明 |
|---|---|---|
| `config` | 自动定位 `default_m20.yaml` | 优先源码 `config/`，其次安装前缀 `lib/lightning/config/`，可显式覆盖为绝对路径 |
| `enable_joints_adapter` | `true` | 是否启动 `/JOINTS_DATA→/joint_states` 桥接（无 drdds 时置 false） |
| `enable_leg_odom` | `true` | 是否启动轮腿里程计 |

门禁话题随 enable 参数动态组装：基础为 `/IMU`、`/LIDAR/POINTS`、`/lio_pose`（定位版为 `/ODOM`），
启用 adapter 才加 `/joint_states`，启用 leg_odom 才加 `/odom_wheel`。任一门禁进程非零退出（超时/失败）
会 `Shutdown` 整条 launch；全部就绪则干净退出并保持链路运行。

### 12.2 导航 launch（m20_navigation）

见 §10.2 参数表；导航链路以 `/ODOM` + `/GRID_MAP`（或 `/NAV_POINTS` fallback）为输入，
`enable_motion_output` 控制是否输出 `/NAV_CMD`。

---

## 13. 性能验收 SOP（双轨制）

先 x86/WSL 仿真基准，真机到位后在 RK3588 按同一指标验收。合格线（对齐 README 声明并放宽）：

- 建图 CPU ≤ 2 核、定位 ≤ 0.5 核；内存 ≤ 1.5GB；LIO 位姿延迟 < 100ms；≥2h RSS 增长 < 10%

采样：

```bash
cd /path/to/light/sim_ws/scripts
./perf_sample.sh run_slam_online 90 /tmp/slam_perf.csv   # 输出 ts,pid,cpu_pct,rss_kb
```

x86/WSL corridor 基准：CPU 均值 ~7.2% / 峰值 ~14.6%，RSS 107.9–141MB，帧间隔 P95=100ms。

若真机性能不达标，按序降级并每次重跑基准对照：

1. `fasterlio.point_filter_num` 调大（先到 6，再 10）
2. 关 `system.with_g2p5`
3. 关 `system.with_loop_closing`
4. 最后降 `fasterlio.scan_line`（96→64→32）

---

## 14. 故障排查

| 现象 | 排查 |
|---|---|
| 门禁 60s 超时并 Shutdown | 查看 launch 日志确认缺哪个话题；组件被禁时对应话题不应出现在门禁列表 |
| launch 报 executable not found | 使用了手工同步的 install：`export PATH=$PWD/install/lightning/lib/lightning:$PATH`，或重新 colcon build |
| `YAML::BadFile` | config 路径不存在；检查 `config:=` 覆盖值或重新安装 config |
| 定位不输出 `/ODOM` | 地图未加载 / 初始位姿错误 / NDT 匹配失败，先 `/initialpose` 重定位 |
| 建图在走廊漂移 | 确认 `/odom_wheel` 有数据且 `odom_topic: /odom_wheel` 生效 |
| 真机 `/joint_states` 无数据 | adapter 未启动或 `drdds` 不可用；核对 `/JOINTS_DATA` |
| 导航规划空 / Rejecting goal | 无 `/GRID_MAP` 时 fallback 地形窗口仅 ±4m，超范围目标必失败；请接入 occ_grid_bridge 提供已知地图 |
| 导航不动但规划正常 | `enable_motion_output=false`（默认安全干跑）；启用 `--motion` 并确认 `/NAV_CMD` 链路 |
| directLine 模式切角穿障 | `local.directLine_mode: true` 无碰撞校验，狭窄场景置 false 走 DWA |
| colcon 在中文路径失败 | rosidl UTF-8 bug，改 ASCII 路径构建（§5.2） |

---

## 15. Git 与数据约定

- `data/`、`build/`、`install/`、`log/`、第三方构建产物均被 `.gitignore` 排除；
  官方数据集（约 1.1GB）**不入库**，按 §6 下载。
- 上游 `lightning-lm/doc/` 内的 gif/png 演示图保留在仓库历史中。

---

## 16. 已知遗留（待真机标定/验证）

| 编号 | 事项 | 状态 |
|---|---|---|
| A | RoboSense 点云 `timestamp` 字段 | ✅ 官方数据已验证存在且布局正确 |
| B | 腿式里程计为单腿 2-DOF 简化模型（协方差放大 100 倍降权） | 待真机数据按需改进 |
| C | `track_width=0.137` 取手册值 | 建议真机实测校准 |
| D | IMU 外参取 identity（驱动已变换到 base_link） | 精度不足时微调 `extrinsic_T/R` |
| E | `/ODOM` 与原厂 NOS localization 双发布冲突 | 部署前 `ros2 topic info /ODOM` 确认发布者；冲突时 `system.odom_topic=/m20_slam/odom`（隔离模式） |
| F | 真机传感器链路（`/LIDAR/POINTS` 未发布、IMU 静默） | 参考 `docs/M20_ALIGNMENT.md` §6，属机器人侧状态，需先恢复 |
| G | 自研导航 footprint 不一致（全局 0.45² vs 局部 0.84×0.5） | 统一机身模型后再真机验收 |
| H | fallback 地形模式（无 `/GRID_MAP`）远程目标不可达 | 真机务必接 occ_grid_bridge / `/GRID_MAP` |
| I | 自研导航尚未真机运动验收（P0-2 directLine、卡死看门狗等） | 按 `docs/M20_DEPLOYMENT.md` 清单修复后在真机验收 |

详细部署步骤与 P0–P2 修复清单见 [docs/M20_DEPLOYMENT.md](docs/M20_DEPLOYMENT.md)；
与本仓库对齐参考（M20 Pro 原厂基线交接文档）的逐项核对见
[docs/M20_ALIGNMENT.md](docs/M20_ALIGNMENT.md)。
方案决策记录见 [docs/M20_SYSTEM_PLAN.md](docs/M20_SYSTEM_PLAN.md)。
