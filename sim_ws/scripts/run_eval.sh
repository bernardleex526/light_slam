#!/bin/bash
# 数据采集 + 评测脚本 (v2: review 修复版)
# 用法: ./run_eval.sh <scene> <baseline|improved> [duration]
#   scene:    corridor | plaza | office
#   version:  baseline（无轮速融合）| improved（轮速融合）
# 产出（唯一命名目录，避免旧进程/旧文件污染）:
#   ${DATA_DIR:-/mnt/d/data}/<scene>_<version>_<ts>/gt.tum         Gazebo ModelStates 真值
#   ${DATA_DIR:-/mnt/d/data}/<scene>_<version>_<ts>/lio.tum        LIO 轨迹（/lio_pose）
#   ${DATA_DIR:-/mnt/d/data}/<scene>_<version>_<ts>/degeneracy.txt 退化状态（nullity+eigenvalues）
#   ${DATA_DIR:-/mnt/d/data}/<scene>_<version>_<ts>/bag/           rosbag2
#   ${DATA_DIR:-/mnt/d/data}/<scene>_<version>_<ts>/logs/          各进程日志
set -e

# 根据脚本位置推导仓库根目录，避免硬编码 /mnt/d/light；可用 LIGHT_ROOT 覆盖
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIGHT_ROOT="${LIGHT_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
SIM_WS="$LIGHT_ROOT/sim_ws"

SCENE=${1:-corridor}
VERSION=${2:-improved}
DUR=${3:-45}
DATA="${DATA_DIR:-/mnt/d/data}"
TS=$(date +%Y%m%d_%H%M%S)
RUN=$DATA/${SCENE}_${VERSION}_${TS}
mkdir -p $RUN/logs

source /opt/ros/humble/setup.bash
source $SIM_WS/install/setup.bash
source $LIGHT_ROOT/install/setup.bash

# 0. 进程卫生：清掉所有遗留仿真/采集进程（Critical 2：旧 lightning 进程会
#    持续写同一输出文件，旧 /joint_states 发布者会污染评测与单元测试）
pkill -9 -f run_slam_online 2>/dev/null || true
pkill -9 -f leg_wheel_odom 2>/dev/null || true
pkill -9 -f 'ros2 bag record' 2>/dev/null || true
pkill -9 -f record_gt 2>/dev/null || true
pkill -9 -f record_lio 2>/dev/null || true
pkill -9 -f record_degeneracy 2>/dev/null || true
pkill -9 -f teleop_sim 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f spawn_entity 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f ring_fill_node 2>/dev/null || true
pkill -9 -f stand_node 2>/dev/null || true
pkill -9 -f spawner 2>/dev/null || true
pkill -9 -f joint_state_broadcaster 2>/dev/null || true
pkill -9 -f diff_drive_controller 2>/dev/null || true
pkill -9 -f controller_manager 2>/dev/null || true
sleep 3

echo "=== run_eval: scene=$SCENE version=$VERSION dur=${DUR}s run=$RUN ==="

# 1. 启动仿真（无 GUI）
nohup ros2 launch m20_description gazebo.launch.py world:=$SCENE \
  > $RUN/logs/launch.log 2>&1 &
LAUNCH_PID=$!
echo "launch pid: $LAUNCH_PID"
sleep 30

# 2. 轮速里程计（两版本共用；注意：必须用 sim 配置 leg_wheel_odom_sim.yaml——
#    真机配置 imu_topic=/IMU（大写）在仿真中不存在，会导致 /odom_wheel 断供、
#    退化走廊 SLAM 发散。2026-08-12 回归修复）
nohup ros2 run leg_wheel_odom leg_wheel_odom_node \
  --ros-args --params-file $LIGHT_ROOT/src/leg_wheel_odom/config/leg_wheel_odom_sim.yaml \
  > $RUN/logs/leg.log 2>&1 &
LEG_PID=$!

# 3. SLAM：improved 用 /odom_wheel 融合，baseline 关掉 odom 输入
if [ "$VERSION" = "improved" ]; then
  CONFIG=$LIGHT_ROOT/lightning-lm/config/default_robosense_sim.yaml
else
  sed 's|odom_topic: "/odom_wheel"|odom_topic: ""|' \
    $LIGHT_ROOT/lightning-lm/config/default_robosense_sim.yaml > /tmp/no_odom.yaml
  CONFIG=/tmp/no_odom.yaml
fi
# use_sim_time: slam.cc 内 node_->set_parameter 已置 true（/lio_pose 时间戳
# 与 /clock GT 同源）。不能加 --ros-args：gflags 会把 --ros-args/-p 当未知
# 命令行标志直接报错退出。
nohup ros2 run lightning run_slam_online --config $CONFIG \
  > $RUN/logs/slam.log 2>&1 &
SLAM_PID=$!
sleep 10

# 4. 轨迹记录（真值 + LIO + 退化状态）
nohup python3 $SIM_WS/scripts/record_gt.py $RUN/gt.tum $DUR \
  > $RUN/logs/gt.log 2>&1 &
GT_PID=$!
nohup python3 $SIM_WS/scripts/record_lio.py $RUN/lio.tum $DUR \
  > $RUN/logs/lio.log 2>&1 &
LIO_PID=$!
nohup python3 $SIM_WS/scripts/record_degeneracy.py $RUN/degeneracy.txt $DUR \
  > $RUN/logs/degen.log 2>&1 &
DEGEN_PID=$!

# 5. rosbag（全量；$RUN 已含唯一时间戳，直接写 $RUN/bag，与头注释/结尾打印一致）
nohup ros2 bag record -o $RUN/bag > $RUN/logs/bag.log 2>&1 &
BAG_PID=$!
sleep 5

# 6. 遥操作轨迹（同一 profile：两版本完全一致）
nohup python3 $SIM_WS/scripts/teleop_sim.py $SCENE > $RUN/logs/teleop.log 2>&1 &
TELEOP_PID=$!
sleep $((DUR + 10))

kill $GT_PID $LIO_PID $DEGEN_PID $BAG_PID $SLAM_PID $LEG_PID $TELEOP_PID 2>/dev/null || true
kill $LAUNCH_PID 2>/dev/null || true
sleep 3
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
pkill -9 -f run_slam_online 2>/dev/null || true
pkill -9 -f leg_wheel_odom 2>/dev/null || true

echo "=== done ==="
echo "GT:    $RUN/gt.tum         ($(wc -l < $RUN/gt.tum) lines)"
echo "LIO:   $RUN/lio.tum        ($(wc -l < $RUN/lio.tum) lines)"
echo "DEGEN: $RUN/degeneracy.txt ($(wc -l < $RUN/degeneracy.txt) lines)"
echo "BAG:   $RUN/bag"
echo "RUN:   $RUN"
