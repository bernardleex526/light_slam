# sim_ws — M20 Gazebo 仿真验证套件

Gazebo Classic 11 + ROS2 Humble 下 M20 机器人的仿真验证环境，用于
leg_wheel_odom + lightning-lm（轮速融合 LIO）的退化场景对比评测。

## 目录

- `src/m20_description/` — M20 机器人模型（URDF 适配 + 控制器 + 传感器）
- `src/ring_fill_node/` — 点云 ring 字段补全节点（/points_raw -> /rs_points）
- `src/gazebo_ros2_control/` — gazebo_ros2_control（humble 源码构建，含修复）
- `worlds/` — 三场景世界（corridor/plaza/office），安装于 m20_description
- `scripts/` — 采集/评测脚本

## 使用

```bash
source /opt/ros/humble/setup.bash
source /path/to/light/sim_ws/install/setup.bash
source /path/to/light/install/setup.bash

# 启动仿真（headless，世界名 corridor/plaza/office）
ros2 launch m20_description gazebo.launch.py world:=corridor

# 数据采集 + 评测（改进版/基线版）
cd /path/to/light/sim_ws/scripts
./run_eval.sh corridor improved 80
./evo_eval.sh corridor improved
```

> 脚本会自动根据 `sim_ws/scripts/run_eval.sh` 的位置推导仓库根目录，
> 也支持用 `LIGHT_ROOT=/path/to/light` 和 `DATA_DIR=/path/to/data` 覆盖。

## 关键话题

- `/joint_states`（4 轮）、`/imu`（200Hz）、`/points_raw`（ray 16 线 PointCloud2）
- `/rs_points`（ring_fill 输出，含 ring 字段，供 lightning）
- `/diff_drive_controller/odom`、`/diff_drive_controller/cmd_vel_unstamped`
- `/odom_wheel`（leg_wheel_odom，50Hz twist）
- `/lio_pose`（lightning LIO 位姿，sim 时间）、`/degeneracy_status`
- `/model_states`（Gazebo 真值，best_effort QoS）

## 已知问题与修复（重要）

1. **apt 版 gazebo_ros2_control 0.4.10 不可用**：其 libgazebo_hardware_plugins.so
   缺少 class_loader 工厂注册，且 plugin 描述只按
   `GazeboSystemInterface` 基类导出（controller_manager 的
   `ClassLoader<SystemInterface>` 无法发现）。已在 `src/gazebo_ros2_control/`
   从源码重建并修复：
   - `gazebo_system.cpp`：追加
     `PLUGINLIB_EXPORT_CLASS(GazeboSystem, hardware_interface::SystemInterface)`
   - `gazebo_hardware_plugins.xml`：追加 SystemInterface 条目
   - `gazebo_ros2_control_plugin.cpp`：`robot_description:=<urdf>` 无法被 rcl
     参数解析器接受（URDF 含引号/换行），改为直接在 controller_manager 节点
     上设置；并把 `<parameters>` yaml 中的控制器参数显式下发（controller
     nodes 不继承插件 context 参数）
2. **站立姿态**：腿关节采用"固定腿回退"方案（`adapt_urdf.py` 将 12 个腿关节
   转为 fixed）——机器人以刚性四轮车形态稳定站立（z=0.590）。仿真中的站立
   姿态（hipy≈-0.7, knee≈1.4 + joint_trajectory_controller）代码保留在
   `adapt_urdf.py`/`stand_node.py` 中，但 ODE 下长时间站立会发散，故默认关闭。
3. **控制器 spawn 竞态**：spawner 必须串行（controller_manager 2.54 并发
   load/configure/activate 会报 "can not be configured from 'active' state"）。
4. **cmd_vel 话题**：`use_stamped_vel: false` 时 diff_drive 订阅
   `~/cmd_vel_unstamped`（即 `/diff_drive_controller/cmd_vel_unstamped`）。
5. **/model_states 为 best_effort**：record_gt.py 必须用 best_effort QoS。
6. **ring 字段**：PCL 类型严格匹配，ring 必须 UINT16（不是 UINT8）。

## 评测结果（corridor, 80s 往返, evo_ape tum -va）

| 版本 | mean APE | rmse | max |
|---|---|---|---|
| baseline（无轮速融合） | 1.71 m | 1.93 m | 3.86 m |
| improved（轮速融合）   | 0.23 m | 0.28 m | 0.55 m |

improved 走廊漂移约为 baseline 的 13%（显著低于 50% 阈值）。
