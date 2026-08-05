#!/bin/bash
# 数据采集 + 评测脚本
# 用法: ./run_eval.sh <scene> <baseline|improved> [duration]
#   scene:    corridor | plaza | office
#   version:  baseline（无轮速融合）| improved（轮速融合）
# 产出:
#   /mnt/d/data/<scene>_<version>/             rosbag2
#   /mnt/d/data/<scene>_gt.tum                 Gazebo ModelStates 真值
#   /mnt/d/data/<scene>_<version>_traj.tum     LIO 轨迹（/lio_pose）
set -e

SCENE=${1:-corridor}
VERSION=${2:-improved}
DUR=${3:-60}
DATA=/mnt/d/data
mkdir -p $DATA

source /opt/ros/humble/setup.bash
source /mnt/d/light/sim_ws/install/setup.bash
source /mnt/d/light/install/setup.bash

pkill -9 -f gzserver 2>/dev/null || true
sleep 2

echo "=== run_eval: scene=$SCENE version=$VERSION dur=${DUR}s ==="

# 1. 启动仿真（无 GUI）
nohup ros2 launch m20_description gazebo.launch.py world:=$SCENE \
  > /tmp/eval_launch.log 2>&1 &
LAUNCH_PID=$!
echo "launch pid: $LAUNCH_PID"
sleep 30

# 2. 轮速里程计（两版本共用）
nohup ros2 run leg_wheel_odom leg_wheel_odom_node \
  --ros-args --params-file /mnt/d/light/src/leg_wheel_odom/config/leg_wheel_odom.yaml \
  > /tmp/eval_leg.log 2>&1 &
LEG_PID=$!

# 3. SLAM：improved 用 /odom_wheel 融合，baseline 关掉 odom 输入
if [ "$VERSION" = "improved" ]; then
  CONFIG=/mnt/d/light/lightning-lm/config/default_robosense_sim.yaml
else
  sed 's|odom_topic: "/odom_wheel"|odom_topic: ""|' \
    /mnt/d/light/lightning-lm/config/default_robosense_sim.yaml > /tmp/no_odom.yaml
  CONFIG=/tmp/no_odom.yaml
fi
nohup ros2 run lightning run_slam_online --config $CONFIG \
  > /tmp/eval_slam.log 2>&1 &
SLAM_PID=$!
sleep 10

# 4. 轨迹记录（真值 + LIO）
nohup python3 /mnt/d/light/sim_ws/scripts/record_gt.py $DATA/${SCENE}_gt.tum $DUR \
  > /tmp/eval_gt.log 2>&1 &
GT_PID=$!
nohup python3 /mnt/d/light/sim_ws/scripts/record_lio.py $DATA/${SCENE}_${VERSION}_traj.tum $DUR \
  > /tmp/eval_lio.log 2>&1 &
LIO_PID=$!

# 5. rosbag（全量）
nohup ros2 bag record -o $DATA/${SCENE}_${VERSION} -a > /tmp/eval_bag.log 2>&1 &
BAG_PID=$!
sleep 5

# 6. 遥操作轨迹
nohup python3 /mnt/d/light/sim_ws/scripts/teleop_sim.py $SCENE > /tmp/eval_teleop.log 2>&1 &
TELEOP_PID=$!
sleep $((DUR + 10))

kill $GT_PID $LIO_PID $BAG_PID $SLAM_PID $LEG_PID 2>/dev/null || true
kill $LAUNCH_PID 2>/dev/null || true
sleep 3
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true

echo "=== done ==="
echo "GT:    $DATA/${SCENE}_gt.tum      ($(wc -l < $DATA/${SCENE}_gt.tum) lines)"
echo "LIO:   $DATA/${SCENE}_${VERSION}_traj.tum ($(wc -l < $DATA/${SCENE}_${VERSION}_traj.tum) lines)"
echo "BAG:   $DATA/${SCENE}_${VERSION}"
