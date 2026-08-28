# M20 完整系统计划（基于 lightning-lm）— 草稿，待 grill

> 工作区：`D:\slam\自研\light`
> 基础：lightning-lm（已完成 M20 P0–P2 环境适配，见 `docs/M20_DEPLOYMENT.md`）+ `src/leg_wheel_odom` + `src/m20_joints_adapter` + `sim_ws`（Gazebo 仿真验证套件）
> 目标硬件：云深处 M20 Pro 机器狗（ROS2 Foxy / RK3588 ARM / GOS 10.21.31.104 或 NOS 10.21.31.106）

## 一、现状（已确认，不再重复决策）

### 已完成
1. **环境适配 P0–P2**（12 项修复，见 M20_DEPLOYMENT.md）：`m20_joints_adapter`（/JOINTS_DATA→/joint_states）、定位节点发布 /ODOM（map 系 10Hz）、`use_sim_time` 可配置、ARM SSE 编译守卫、Foxy 依赖脚本、occ_grid.pgm/yaml 输出（resolution 可配）、/initialpose 重定位、default_m20.yaml 真机配置、打滑检测（IMU 偏航 vs 轮式差速）、节点改名 lightning_loc 等。
2. **仿真验证**（sim_ws）：Gazebo 三场景（corridor/plaza/office）、ring_fill_node（ring 字段补全）、gazebo_ros2_control 源码构建修复；corridor 80s 往返评测：轮速融合改进版 APE 0.23m vs 基线 1.71m（约 13%）。
3. **部署资产**：`Dockerfile.foxy`、`scripts/install_dep.sh`（自动识别 Foxy/Humble）、sim 配置齐全。

### 已知遗留（需真机标定/验证）
- **A. RoboSense 点云 timestamp 字段**：`PointRobotSense` 需要点云含 timestamp（double, 秒），真机需验证 `/LIDAR/POINTS` 字段名。
- **B. 腿式里程计模型简化**：仅 FL 腿 2-DOF 平面模型，忽略 hipx 与多腿融合；协方差放大 100 倍降权，真机建议 LIO 为主。
- **C. track_width 0.137**：取手册 W_hip，需真机实测校准。
- **D. IMU 外参 identity**：驱动已变换到 base_link，定位精度不足时微调 extrinsic_T/R。

## 二、目标（本次 grill 聚焦）

面向云深处 M20 的完整系统，四大支柱：

1. **环境适配** — 主体已完成（P0–P2），剩余：真机链路验证 + 遗留 A–D。
2. **模型部署** — lightning-lm 软件栈在 M20（RK3588 ARM / Foxy）上的部署形态与方式（范围待决策）。
3. **功能集成** — 与 M20 官方系统的集成深度：drmap 地图管理、官方 planner、/ODOM、/initialpose（深度待决策）。
4. **可用性与性能** — RK3588 上 CPU/内存/延迟验收标准、长时间运行稳定性、启动与运维方式（标准待决策）。

## 三、决策树（待 grill）

```
1. ✅ [parent] "模型部署"范围：仅 lightning-lm 软件栈本身（建图+定位+里程计融合），不含额外 AI 感知模型。（user confirmed；D2 子节点已消除）
3. ✅ [parent] 真机验证：**M20 真机当前不可用**。验证结构改为：仿真（已完成）→ 真机到位后：录包离线验证 → 真机在线验证；真机相关项全部转为"待真机 SOP + 验收标准"。（user confirmed）
4. ✅ [parent] 功能集成深度：**含官方导航闭环**（occ_grid → drmap apply → 官方 planner 端到端跑通）。（user confirmed）
5. ✅ [parent] leg_wheel_odom 改进：**不做多腿融合改进**，保留简化模型，列为真机到位后按需项。（user confirmed）
6. ✅ [parent] 部署包装：**源码+安装脚本（现状路线），Docker 不投入**（Dockerfile.foxy 保留为可选资产）。（user confirmed）
7. ✅ [parent] 性能验收：**双轨制**——现在建仿真基准（x86/WSL corridor），真机到位后 RK3588 验收；及格线 = x86 声称值×2（建图 ≤2核、定位 ≤1.5核、内存 ≤1.5GB、延迟 <100ms、2h RSS 增长 <10%）。（user confirmed）
8. ✅ [child of 7] 降级预案顺序：**point_filter_num 调大 → 关 g2p5 → 关回环 → 最后降 scan_line**，每次降级后重跑基准对照。（user confirmed）
9. ✅ [parent] 运维化：**ROS2 launch 一键启动（建图/定位两套）+ 日志落盘；不做 systemd/独立看门狗**；launch 内置 topic 就绪门禁。（user confirmed）
10. ✅ [child of 3] 真机录包离线验证 = 真机到位后第一验证门；真机到位前的预验证数据源：**官方云深处数据集（DeepRoboticsLab/lightning-lm-deep-robotics + 百度云多层数据）做离线建图/定位验证**，取不到则退回仅仿真。（user confirmed）
```

## 四、工作分解（决策已定）

### ✅ W1 官方数据集离线验证（已完成 2026-08-12）
- **数据**：DeepRoboticsLab office bag（45.3s，/LIDAR/POINTS 10Hz + /IMU 183Hz，无 odom/tf），持久化于 `data/m20_office_bag/`
- **建图**：2026-08-26 全新目录复测 run_slam_offline + default_m20.yaml EXIT=0，162 KF、154,432 点，
  53 次约 3ms 小回退被单调时间门限修正、0 帧因时钟故障丢弃，occ_grid.pgm/yaml 正常输出
- **定位**：2026-08-26 使用上述新地图首次运行 run_loc_offline，165/166 帧匹配成功、
  confidence 均值 2.983；53 次约 3ms 小回退被修正、0 帧因时钟故障丢弃。定位会写回
  dynamic map，因此基准必须从全新建图目录开始，不能在同一地图目录反复定位后比较统计
- **遗留 A 结论**：timestamp 字段存在且布局正确（double@offset18），frame_id=lidar_link；无阻塞
- **发现并修复**：① default_m20.yaml 缺 `plane_icp_weight`/`proj_kfs` 必需键（已补回）；② lidar_loc.cc:850 调试残留 savePCDFile 致 loc 崩溃（已注释）；③ slam.cc occ_grid yaml_path_ 编译错误（已修）；④ **构建链：工作区路径含中文"自研"，colcon 在此路径下不可用**（rosidl UTF-8 bug），需在 ASCII 路径（/tmp/light_src）构建后同步产物——环境级约束，待解决
- 性能（离线全速，仅供参考）：CPU 均值 ~127%/峰值 ~170%，RSS 峰值 ~205MB

### ✅ W2 性能仿真基准（已完成 2026-08-12）
- **发散回归修复验证通过**：corridor improved 重跑一次即无发散，evo APE 从修复前 165–608m（发散）恢复到 **median 0.162m / mean 0.165m / max 0.311m**（修复前记录值 0.23m 量级一致）；退化警告 312→53 次（-83%）；轨迹 19.79m vs GT 19.76m
- **性能数据（修复后复用，达标）**：CPU 均值 ~7.2%/峰值 ~14.6%（≈0.07–0.15 核，远低于 README x86/32 线 1.2 核声称）；RSS 107.9–141MB（合格线 <1.5GB）；帧间隔 P95=100ms（边界通过，注意 0.81× 实时率下为标称节拍）
- 遗留观察：记录窗口外静止 ~42s 后单帧漂移（sim>100s，评测窗外）——静止退化场景潜在风险，真机验收时关注
- 附带修复：run_eval.sh bag 记录目录冲突（`-o $RUN/bag_$TS`）
- 降级预案验证（D8 顺序）未执行——基准已达标，无降级需求；预案留档待真机需要时用

### ✅ W3 运维与启动（已完成 2026-08-12）
- 两套 launch：`m20_slam.launch`（adapter + leg_wheel_odom + run_slam_online）、`m20_loc.launch`（定位版，就绪话题用 /ODOM）
- `topic_ready_check.py` 就绪门禁：非 0 退出才 Shutdown，BEST_EFFORT+reliable 双兼容，动态解析话题类型
- CMakeLists + package.xml 安装声明；py_compile 通过
- 待办：M20_DEPLOYMENT.md 部署步骤引用 launch（真机到位前补即可）

### W4 待真机 SOP（真机到位后按序执行，D3/D4/D5）
- 门 1：录包 → 离线建图/定位验证（D10）
- 门 2：在线验证（验证清单：/joint_states、/odom_wheel、/ODOM、/IMU、/lio_pose）+ 遗留 A 真机确认
- 门 3：官方导航闭环（occ_grid → drmap apply → 官方 planner → ±0.3m 到达）
- 门 4：RK3588 性能验收（W2 SOP）
- 按需项（不承诺）：腿式模型多腿融合（D5）、track_width 实测、IMU 外参微调（遗留 C/D）

### 明确不做（D1/D4/D5/D6/D9）
- 额外 AI 感知模型；Docker 部署投入；多腿融合改进（本次）；systemd/独立看门狗

---

## Resolved Decisions

1. **"模型部署"范围** — 仅 lightning-lm 软件栈本身（建图+定位+里程计融合），不含额外 AI 感知模型。Why: 全仓库无推理框架/模型工件引用；云深处官方 GOS/NOS 自带感知避障，"模型部署"= 将 SLAM 系统部署至目标硬件。（user confirmed；子节点 D2 随之消除）
2. **真机可用性** — M20 真机**当前不可用**。验证结构改为三阶段：仿真（已完成）→ 真机到位后录包离线验证 → 真机在线验证；全部真机相关项（W1、W4、遗留 A/B/C/D）转为"待真机 SOP + 验收标准"，真机到位即按序执行。（user confirmed；D10 变形为"真机到位后第一验证门"）
3. **功能集成深度** — 含官方导航闭环：occ_grid.pgm/yaml → drmap unpack/apply → 官方 planner 端到端跑通（/ODOM 供给）。Why: /ODOM 发布器与 occ_grid 输出即为官方 planner 而做（M20_DEPLOYMENT.md P0#2/P1#11），map_path 已对齐 drmap active 目录；不做闭环则适配无验收意义。（user confirmed）
4. **leg_wheel_odom 模型** — 不改进（保留单腿 2-DOF + 协方差×100 降权），列为真机到位后按需项。Why: 无真机数据无法验证；仿真腿关节为 fixed（sim_ws README §2），腿式路径测不到；现有降权设计已兜底退化场景。（user confirmed）
5. **部署包装** — 源码+安装脚本（现状路线，M20_DEPLOYMENT.md 已文档化），Docker 不投入。Why: RK3588 为 ARM64，上游 demo 镜像为 x86 不可用，ARM 自建成本高（Pangolin 手动编译 + drdds 依赖 + 构建脆弱）；轮速融合链路本就不在容器内。（user confirmed；Dockerfile.foxy 保留为可选资产）
6. **性能验收（双轨制）** — 现在：x86/WSL 仿真基准（corridor，定位+建图 CPU/内存/延迟）；真机到位后：RK3588 按同一指标 SOP 验收。及格线 = README x86 声称值×2：建图 CPU ≤2 核、定位 ≤1.5 核、内存 ≤1.5GB、lio 位姿延迟 <100ms、≥2h RSS 增长 <10%。Why: ARM 弱于 x86 且 96 线 > 32 线，×2 放宽为合理保守线。（user confirmed）
7. **性能降级预案（顺序）** — ① `point_filter_num` 调大（6→8/10，README 明示旋钮，代价最小）→ ② 关 g2p5（定位阶段本不需实时栅格）→ ③ 关回环（仅影响建图后端精度）→ ④ 最后降 `scan_line`（96→64，硬件语义级降级影响定位最重）。每次降级后重跑基准对照。（user confirmed）
8. **运维化** — ROS2 launch 一键启动（建图/定位两套），launch 内置 topic 就绪门禁（adapter/odom/IMU/lidar 存在性检查）+ 日志落盘。不做 systemd、不做独立监控看门狗。Why: M20 跑官方 GOS/NOS 自带进程管理，插 systemd 有冲突风险且真机不可用无法验证；现状手动步骤的最小补齐即 launch + 日志。（user confirmed）
9. **真机前预验证数据源** — 官方云深处数据集（`DeepRoboticsLab/lightning-lm-deep-robotics` + README 百度云多层数据）跑离线建图/定位。Why: config 已对齐（robosense lidar_type=4，default_m20.yaml:20），数据集与配置路径完全匹配，可提前暴露 timestamp 等真机侧问题；取不到则退回仅仿真。（user confirmed）
10. **真机到位后验证门（D10 结论）** — 第一验证门 = 真机录包 → 离线建图/定位验证（先于在线验证）。（user confirmed）
