# light_slam deployment contract and Humble bringup

This patch fixes the audited fusion, map-loading and ROS interfaces. It does not establish binary equivalence with the M20 factory algorithm. Validated development platform: Ubuntu 22.04 / ROS 2 Humble / x86_64 WSL2. M20 Pro ARM64/Foxy and other vendors still require native builds and hardware acceptance. Do not assume a generic PointCloud2 message has the required per-point time encoding.

## Build

Use the updated repository checkout. The validation baseline before these fixes was `fbfc7d59202fa3e80b140e3e557016ec8b71b5fe`. Use the repository's existing Pangolin/PCL/Eigen/g2o dependencies. Install ROS dependencies with rosdep for the selected packages, with `M20_HAS_DRDDS=0` on development hosts. The proprietary drdds SDK is optional and is not included.

```bash
source /opt/ros/humble/setup.bash
export M20_HAS_DRDDS=0
colcon build --packages-select lightning leg_wheel_odom m20_joints_adapter m20_navigation \
  --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
colcon test --packages-select lightning leg_wheel_odom m20_navigation
colcon test-result --verbose
```

## Robot configuration

Copy an installed `lightning/config/robots/*.yaml` template to a writable location. `m20pro.yaml` preserves the repository's M20 sensor topics; `generic_robosense.yaml` uses `/points_raw` and `/imu/data`. Both require RoboSense-format point fields and timestamps. For Livox/Velodyne and other supported encodings, transfer the relevant preprocessing settings from their existing configs; changing the topic name is insufficient.

- IMU: SI units (m/s², rad/s), sensor measurement timestamps in the same clock as LiDAR. Accelerometer data includes gravity. Verify axis handedness.
- `fasterlio.extrinsic_R/T`: transform a LiDAR point into IMU coordinates. Keyframe clouds are transformed once into IMU coordinates before map/loop processing.
- `system.imu_from_base_translation` and quaternion `xyzw`: pose of base_link in the IMU frame. Identity is a placeholder, not measured M20 calibration.
- `system.map_frame/odom_frame/base_frame`: distinct frame names. Publish calibrated static sensor transforms through the robot description. Only light_slam should publish `map->odom` and `odom->base_link` in this configuration.
- `system.sensor_best_effort`: match the sensor publisher. Use false for the tested reliable rosbag replay; true accepts best-effort hardware publishers.
- `system.use_sim_time`: false on hardware; true for rosbag `--clock`. Restart SLAM before replaying from an earlier time. Stale LiDAR samples are rejected.
- `g2p5` floor/height thresholds and Nav2 obstacle heights must match the calibrated vertical reference. Defaults are not universal for quadrupeds.
- Auxiliary odometry is disabled in new profiles. To enable, set `common.odom_topic` and a positive `wheel_odom_weight` only after validation. Input twist must refer to the **IMU origin, expressed in IMU axes**, with sample timestamps bracketing each LiDAR update; gaps over 150 ms are rejected. For base twist use `omega_i=R_ib*omega_b`, `v_i=R_ib*v_b-omega_i.cross(t_ib)`. This planar auxiliary constraint is inappropriate for arbitrary airborne/slipping legs without a contact/slip estimator.

## Mapping and localization

```bash
ros2 launch lightning robot.launch.py mode:=mapping config:=/absolute/robot.yaml work_dir:=/absolute/maps_workspace
ros2 service call /lightning/save_map lightning/srv/SaveMap '{map_id: trial_01}'
```

Save output is `/absolute/maps_workspace/data/trial_01/`, including global PCD, tiled map and `occ_grid.yaml/pgm`. Set `system.map_path` to that saved directory before localization. A save request before the first keyframe returns an error.

```bash
ros2 launch lightning robot.launch.py mode:=localization config:=/absolute/robot.yaml work_dir:=/absolute/maps_workspace
```

Localization starts searching near identity; send a calibrated map-frame base pose on `/initialpose` if needed. `/odom` is continuous LIO odometry, `/ODOM` and `/lio_pose` are map-corrected base poses. All outputs use measurement time, with body-frame twist in Odometry. `map->odom` comes from matching global and local estimates at the same time; it is not a second independent base transform.

## Nav2

Start localization and calibrated sensor TF first, then:

```bash
ros2 launch m20_navigation light_slam_nav2.launch.py map:=/absolute/maps_workspace/data/trial_01/occ_grid.yaml
```

The launch uses standard Nav2, map server and light_slam TF; it does not start AMCL or the repository's separate factory-style planner. Override `params_file` for another robot. The included footprint is the repository's M20 body rectangle plus 5 cm padding; measure the complete moving-leg envelope. Edit both costmaps, input cloud topic, ground-relative height thresholds, acceleration and velocity limits. The Regulated Pure Pursuit local controller checks collisions against the costmap footprint; unknown-space global planning is disabled. This initial profile uses forward motion and turning, not lateral stepping. Goal tolerances are 0.25 m and 0.30 rad; adapt these to task needs.

Nav2 emits standard `geometry_msgs/Twist` on `/cmd_vel`. A robot with an existing compatible ROS velocity interface can consume it. Other robots require a vendor-specific adapter translating this into their SDK, gait/mode and stop/watchdog commands. The existing native navigation bridge belongs to the custom planner; it is not an automatically verified Nav2-to-M20 command adapter. Do not run competing command producers. No physical commands were sent during validation.

## Acceptance boundaries

Recorded bag playback tests estimation and interfaces; it cannot test closed-loop navigation because robot motion cannot respond to the controller. Independent surveyed/ground-truth sequences are needed for ATE/RPE and map accuracy. Physical readiness requires the actual robot's calibration, timestamps/QoS, valid obstacle cloud/TF, native build, velocity watchdog, and low-speed goal/stop/obstacle tests. Neither same-map localization nor a static planned path proves those outcomes.

Controller reference: [Nav2 Humble Regulated Pure Pursuit](https://api.nav2.org/nav2-humble/html/md_nav2_regulated_pure_pursuit_controller_README.html).
