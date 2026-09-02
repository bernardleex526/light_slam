# config/native/ —— M20 Pro 原厂参数快照（只读参考）

本目录是 **M20 Pro 原厂闭源节点**的可观测参数快照，来自
`m20_orignal` 仓库（基于《M20 Pro 原厂基线交接文档》即"对齐参考"整理），
**只读参考，不参与 light_slam 运行加载**。用途：

1. **对接原厂导航时的契约核对**：原厂 NOS planner 订阅哪些话题、用什么参数，
   据此确认 `light_slam` 的 `/ODOM`、`occ_grid` 输出与原厂链路匹配。
2. **机体/雷达标定参考**：`native_lidar_params.yaml`、`native_body_params.yaml`
   给出原厂前/后雷达装体位姿与机身尺寸（0.82×0.43×0.5 m），可对照
   `default_m20.yaml` 中的 `ui.car_length/width` 与 LIO 外参假设。

## 文件来源

| 文件 | 原厂路径 |
|---|---|
| `native_global_topics.yaml` | `/opt/robot/share/global_planner/config/config.yaml` |
| `native_global_planner.yaml` | `/opt/robot/share/global_planner/config/config_astar.yaml` |
| `native_local_planner.yaml` | `/opt/robot/share/planner/config/localplanner.yaml` |
| `native_passable_area.yaml` | `/opt/ros/foxy/share/passable_area/config/nav_params.yaml` |
| `native_pcl_pass_grid.yaml` | `/opt/robot/share/planner/config/pcl_pass_grid.yaml` |
| `native_lidar_params.yaml` | `/opt/ros/foxy/share/passable_area/config/lidar_params.yaml` |
| `native_body_params.yaml` | `/opt/ros/foxy/share/passable_area/config/body_params.yaml` |

> 注：m20_orignal 另有 `native_navigation.yaml`（其自研导航节点配置）。本仓库选择
> "只对接原厂导航"路线（不移植自研导航），故不包含该文件。

## 与原厂导航对接的关键契约（摘自 native_global_topics.yaml）

| 话题 | 类型 | light_slam 侧 |
|---|---|---|
| `/GRID_MAP` | `nav_msgs/OccupancyGrid` | 原厂 NOS 发布，light_slam 不消费 |
| `/ODOM` | `nav_msgs/Odometry` | **light_slam 定位发布**（`system.odom_topic`，默认 `/ODOM`） |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 原厂 planner 输入，light_slam 不消费 |
| `/NAV_POINTS` | `sensor_msgs/PointCloud2` | 原厂 planner 局部点云输入，light_slam 不消费 |
| `/NAV_CMD` | `drdds/msg/NavCmd` | 原厂运动输出，light_slam **不写**（安全边界） |