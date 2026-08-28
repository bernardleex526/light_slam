# M20 SLAM 系统（lightning-lm + 轮腿里程计）

本仓库是面向云深处 **M20 Pro** 轮足机器狗（ROS2 Foxy / RK3588 ARM）的激光-惯导建图与定位系统。
核心算法为开源 [lightning-lm](https://github.com/gaoxiang12/lightning-lm)（Lightning-Speed Lidar
Localization and Mapping），本仓库在其上完成了 M20 真机适配、轮腿里程计融合、一键启动与仿真/性能验证闭环。

## 1. 系统能力总览

- **前端 LIO（FASTER-LIO 风格）**：12 维 ESKF（位置/姿态/速度/陀螺零偏）+ ivox3d 增量体素地图，
  点面 ICP（主）+ 点对点 ICP（可选），支持 Livox / Velodyne / Ouster / RoboSense 四种雷达。
- **轮速/腿式里程计融合**：自研 `leg_wheel_odom` 节点输出 `/odom_wheel`（50Hz），在 ESKF 中以帧间
  SE3 增量约束 x/y/yaw，退化场景（如长走廊）自动加大轮速权重，显著抑制漂移。
- **后端回环**：grid-NDT 回环检测 + 自研轻量图优化库 `miao`（g2o 风格，GN/LM/DogLeg）。
- **地图**：分块点云地图（静态层 + 动态层，动态层支持短期/长期/永久策略），`g2p5` 实时输出
  2.5D 栅格地图 `occ_grid.pgm/yaml`（兼容 M20 官方 drmap）。
- **定位**：LIO 里程计 + NDT-OMP 地图匹配（pclomp）+ PGO 位姿图多源融合，输出 IMU 同频位姿
  （100Hz）、`/ODOM`（map 系，10Hz）与 `map→base_link` TF，支持 `/initialpose` 重定位。
- **一键启动**：`m20_slam.launch.py` / `m20_loc.launch.py`，带话题就绪门禁（详见 §12）。
- **仿真验证**：`sim_ws` 提供 M20 Gazebo 模型与 corridor/plaza/office 三场景及评测脚本。

## 2. 目录结构

| 路径 | 说明 |
|---|---|
| `lightning-lm/` | 核心 SLAM 包：`src/core/lio` 前端、`loop_closing` 回环、`g2p5` 栅格、`maps` 分块地图、`miao` 图优化、`system` 系统接口 |
| `lightning-lm/launch/` | `m20_slam.launch.py`、`m20_loc.launch.py`、`topic_ready_check.py` 就绪门禁 |
| `lightning-lm/config/default_m20.yaml` | M20 真机默认配置（96 线 RoboSense、关 UI、QoS 等） |
| `lightning-lm/docker/Dockerfile.foxy` | Foxy 镜像定义（可选资产） |
| `src/m20_joints_adapter/` | `/JOINTS_DATA`(drdds) → `/joint_states` 桥接（真机必需） |
| `src/leg_wheel_odom/` | 轮腿里程计节点（`/odom_wheel`，50Hz） |
| `sim_ws/` | Gazebo 仿真验证套件（M20 模型、ring 补全、三场景、采集/评测脚本） |
| `docs/` | `M20_DEPLOYMENT.md`（真机部署）、`M20_SYSTEM_PLAN.md`（方案决策记录） |
| `data/` | 数据/地图目录（**不入库**，见 §15） |

## 3. 传感器与话题约定（M20）

| 话题 | 方向 | 说明 |
|---|---|---|
| `/LIDAR/POINTS` | 输入 | RoboSense 96 线点云，10Hz，含 `timestamp` 字段 |
| `/IMU` | 输入 | 200Hz |
| `/JOINTS_DATA` | 输入 | 真机 drdds 自定义消息，由 adapter 桥接 |
| `/joint_states` | 内部 | adapter 输出，16 关节 |
| `/odom_wheel` | 输入 | leg_wheel_odom 输出，50Hz |
| `/lio_pose` | 输出 | 建图时 LIO 位姿（geometry_msgs/PoseStamped） |
| `/ODOM` | 输出 | 定位位姿（map 系 nav_msgs/Odometry，10Hz） |
| `/initialpose` | 输入 | 重定位接口（PoseWithCovarianceStamped） |
| `/lightning/save_map` | 服务 | 保存地图 |

## 4. 环境与依赖

真机：Ubuntu 20.04 + ROS2 Foxy + RK3588（GOS 10.21.31.104 / NOS 10.21.31.106）。
开发机：Ubuntu 20.04/22.04 + ROS2 Foxy/Humble。

```bash
cd /path/to/light/lightning-lm
bash scripts/install_dep.sh   # 自动识别 Foxy/Humble，apt 安装 PCL/OpenCV/yaml-cpp/glog 等
# Pangolin 需手动编译安装（见 docker/Dockerfile.foxy 中 Pangolin 段）
```

如需用 rosdep 统一装包（`drdds` 是 M20 私有消息包、不在 rosdep 索引，必须显式跳过）：

```bash
rosdep install --from-paths . src --ignore-src -y --skip-keys drdds
```

## 5. 构建 SOP

### 5.1 开发机（Humble）

```bash
cd /path/to/light
source /opt/ros/humble/setup.bash
colcon build --packages-select lightning leg_wheel_odom m20_joints_adapter
source install/setup.bash
```

开发机无 `drdds` 时，`m20_joints_adapter` 会被 CMake `find_package(drdds QUIET)` 优雅跳过，不影响其余包。

### 5.2 真机（Foxy / RK3588）

```bash
source /opt/robot/scripts/setup_ros2.sh   # 提供 drdds 等机器人私有消息包
export ROS_DOMAIN_ID=0
cd /path/to/light
colcon build --packages-select lightning leg_wheel_odom m20_joints_adapter
source install/setup.bash
```

> **重要约束**：工作区路径含中文（如 `…/自研/light`）时 colcon 会因 rosidl UTF-8 bug 失败。
> 若必须放在中文路径下，请在 ASCII 路径（如 `/tmp/light_src`）构建后同步产物到本目录。
> 手工同步的 `install` 可能缺少标准 colcon 的 `local_setup` PATH 注入，此时需手动
> `export PATH=$PWD/install/lightning/lib/lightning:$PATH`（标准 `colcon build` 无需此步）。

## 6. 数据集准备（离线验证）

云深处官方 M20 office 数据包（推荐，与本仓库 `default_m20.yaml` 完全对齐）：
仓库 `DeepRoboticsLab/lightning-lm-deep-robotics`；下载后放到 `data/m20_office_bag/`，包含：

- `/LIDAR/POINTS`：10Hz PointCloud2，fields `x,y,z,intensity,ring,timestamp(double)`
- `/IMU`：183Hz

其余公开数据集（NCLT / VBR / UrbanLoco 等）见 `lightning-lm/README_CN.md`。

## 7. 离线验证 SOP

```bash
source install/setup.bash
cd /path/to/light/lightning-lm

# 7.1 离线建图（自动保存至 data/new_map/）
ros2 run lightning run_slam_offline \
  --input_bag ../data/m20_office_bag --config ./config/default_m20.yaml

# 7.2 离线定位（map_path 在 config 里指定，如 data/new_map/）
ros2 run lightning run_loc_offline \
  --input_bag ../data/m20_office_bag --config ./config/default_m20.yaml
```

官方 office bag 当前基准（2026-08-26，`default_m20.yaml`，全新目录单次复测）：452 帧点云
生成 162 个关键帧，全局 PCD 154,432 点；使用该地图首次定位 165/166 帧匹配成功，
confidence 均值 2.983。建图与定位各自都修正了 53 次约 3 ms 的源点云
`header.stamp` 小回退，均无超过 20 ms 的时钟故障丢帧。并行后端收尾可能使全局 PCD
点数在重复运行间小幅变化；关键验收项是进程正常结束、关键帧数、地图文件完整性、时间门限
统计和定位成功率，而不是要求 PCD 点数逐点完全一致。

## 8. 在线建图 SOP（真机）

### 8.1 一键启动（推荐）

```bash
source install/setup.bash
ros2 launch lightning m20_slam.launch.py
# 开发机无 drdds 时 joints_adapter 会自动跳过；如也不接轮速，可加：
# ros2 launch lightning m20_slam.launch.py enable_leg_odom:=false
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

## 9. 在线定位 SOP（真机）

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

## 10. 官方导航对接 SOP（occ_grid → drmap → planner）

1. 建图保存后，`occ_grid.pgm + occ_grid.yaml` 已与 `map_path` 对齐 M20 drmap 激活目录
   （`default_m20.yaml` 中 `system.map_path: /var/opt/robot/data/maps/active/`）。
2. 用 drmap `unpack/apply` 或手动放置地图到 `/var/opt/robot/data/maps/`。
3. 启动本仓库定位链路（§9），官方 planner 订阅 `/ODOM` 即可端到端跑通导航。

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

## 12. launch 参数与就绪门禁

| 参数 | 默认 | 说明 |
|---|---|---|
| `config` | 自动定位 `default_m20.yaml` | 优先源码 `config/`，其次安装前缀 `lib/lightning/config/`，可显式覆盖为绝对路径 |
| `enable_joints_adapter` | `true` | 是否启动 `/JOINTS_DATA→/joint_states` 桥接（无 drdds 时自动跳过） |
| `enable_leg_odom` | `true` | 是否启动轮腿里程计 |

门禁话题随 enable 参数动态组装：基础为 `/IMU`、`/LIDAR/POINTS`、`/lio_pose`（定位版为 `/ODOM`），
启用 adapter 才加 `/joint_states`，启用 leg_odom 才加 `/odom_wheel`。任一门禁进程非零退出（超时/失败）
会 `Shutdown` 整条 launch；全部就绪则干净退出并保持链路运行。

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

## 14. 故障排查

| 现象 | 排查 |
|---|---|
| 门禁 60s 超时并 Shutdown | 查看 launch 日志确认缺哪个话题；组件被禁时对应话题不应出现在门禁列表 |
| launch 报 executable not found | 未 source install/setup.bash 或使用了手工同步的 install：请重新 `source install/setup.bash` 或重新 colcon build |
| `YAML::BadFile` | config 路径不存在；检查 `config:=` 覆盖值或重新安装 config |
| 定位不输出 `/ODOM` | 地图未加载 / 初始位姿错误 / NDT 匹配失败，先 `/initialpose` 重定位 |
| 建图在走廊漂移 | 确认 `/odom_wheel` 有数据且 `odom_topic: /odom_wheel` 生效 |
| 真机 `/joint_states` 无数据 | adapter 未启动或 `drdds` 不可用；核对 `/JOINTS_DATA` |
| colcon 在中文路径失败 | rosidl UTF-8 bug，改 ASCII 路径构建（§5.2） |

## 15. Git 与数据约定

- `data/`、`build/`、`install/`、`log/`、第三方构建产物均被 `.gitignore` 排除；
  官方数据集（约 1.1GB）**不入库**，按 §6 下载。
- 上游 `lightning-lm/doc/` 内的 gif/png 演示图保留在仓库历史中。

## 16. 已知遗留（待真机标定/验证）

| 编号 | 事项 | 状态 |
|---|---|---|
| A | RoboSense 点云 `timestamp` 字段 | ✅ 官方数据已验证存在且布局正确 |
| B | 腿式里程计为单腿 2-DOF 简化模型（协方差放大 100 倍降权） | 待真机数据按需改进 |
| C | `track_width=0.137` 取手册值 | 建议真机实测校准 |
| D | IMU 外参取 identity（驱动已变换到 base_link） | 精度不足时微调 `extrinsic_T/R` |

详细部署步骤与 P0–P2 修复清单见 [docs/M20_DEPLOYMENT.md](docs/M20_DEPLOYMENT.md)，
方案决策记录见 [docs/M20_SYSTEM_PLAN.md](docs/M20_SYSTEM_PLAN.md)。
