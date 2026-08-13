# m20_joints_adapter

Bridges the M20 robot's `/JOINTS_DATA` topic (`drdds/msg/JointsData`) to the
standard `/joint_states` topic (`sensor_msgs/msg/JointState`) that the
`leg_wheel_odom` node subscribes to.

## Why this is needed

The `leg_wheel_odom` node subscribes to `/joint_states`, which is a Gazebo-only
topic that does **not** exist on the real robot. The real M20 robot publishes
joint data on `/JOINTS_DATA` using the custom message type `drdds/msg/JointsData`.
This adapter translates each `JointsData` message into a `JointState` message so
`leg_wheel_odom` (and any other `joint_states` consumer) works unchanged on the
real robot.

## Message mapping

`drdds/msg/JointsData` contains 16 `JointData` entries (index 0-15). Each entry's
`position`, `velocity`, and `torque` are copied into the `JointState` `position`,
`velocity`, and `effort` arrays respectively. The `JointState` header stamp is
taken from the `JointsData` header stamp.

### Joint index -> name mapping (M20 official, doc 21_关节.html)

| Index | M20 joint name      | joint_states name |
|------:|---------------------|-------------------|
|     0 | LeftFrontHipX       | `fl_hipx_joint`   |
|     1 | LeftFrontHipY       | `fl_hipy_joint`   |
|     2 | LeftFrontKnee       | `fl_knee_joint`   |
|     3 | LeftFrontWheel      | `fl_wheel_joint`  |
|     4 | RightFrontHipX      | `fr_hipx_joint`   |
|     5 | RightFrontHipY      | `fr_hipy_joint`   |
|     6 | RightFrontKnee      | `fr_knee_joint`   |
|     7 | RightFrontWheel     | `fr_wheel_joint`  |
|     8 | LeftBackHipX        | `hl_hipx_joint`   |
|     9 | LeftBackHipY        | `hl_hipy_joint`   |
|    10 | LeftBackKnee        | `hl_knee_joint`   |
|    11 | LeftBackWheel       | `hl_wheel_joint`  |
|    12 | RightBackHipX       | `hr_hipx_joint`   |
|    13 | RightBackHipY       | `hr_hipy_joint`   |
|    14 | RightBackKnee       | `hr_knee_joint`   |
|    15 | RightBackWheel      | `hr_wheel_joint`  |

## Parameters

| Parameter              | Default         | Description                          |
|------------------------|-----------------|--------------------------------------|
| `joints_data_topic`    | `/JOINTS_DATA`  | Input topic (`drdds/msg/JointsData`) |
| `joint_states_topic`   | `/joint_states` | Output topic (`sensor_msgs/msg/JointState`) |

## Building

This package depends on the `drdds` custom message package, which is **not** part
of this workspace. On the real robot it is available via:

```bash
source /opt/robot/scripts/setup_ros2.sh
```

On dev machines, `drdds` must be built from the `sdk_deploy` repo. If `drdds` is
absent, `CMakeLists.txt` uses `find_package(drdds QUIET)` and gracefully skips
building the node (this is expected on sim/dev machines).

`drdds` is a private package without a rosdep rule, so any rosdep command must
skip it explicitly:

```bash
rosdep install --from-paths src --ignore-src -y --skip-keys drdds
```

Build as usual from the workspace root:

```bash
colcon build --packages-select m20_joints_adapter
```

## Running

```bash
# Source the workspace (and /opt/robot on the real robot) first.
ros2 run m20_joints_adapter joints_adapter_node
```

Or with a config file:

```bash
ros2 run m20_joints_adapter joints_adapter_node \
  --ros-args --params-file src/m20_joints_adapter/config/joints_adapter.yaml
```

Verify the bridge is working:

```bash
ros2 topic echo /joint_states
```
