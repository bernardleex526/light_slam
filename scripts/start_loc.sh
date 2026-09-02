#!/usr/bin/env bash
# M20 Pro 定位 + 原厂导航对接一键启动（对齐参考 §11 接入方式）
#
# 在 AOS(192.168.101.36) 板上运行：source setup_ros2.sh(Foxy/域0) -> 启动
# joints_adapter + leg_wheel_odom + run_loc_online -> 就绪门禁（以 /ODOM 就绪为标志）
#
# 用法:
#   ./scripts/start_loc.sh [--config PATH] [--no-joints] [--no-leg-odom] [--domain ID]
#
# 说明:
#   - 默认发布 /ODOM（map 系 nav_msgs/Odometry，10Hz，system.odom_topic 可改隔离话题
#     如 /m20_slam/odom）+ map->base_link TF（system.pub_tf=false 可关 TF 只留 /ODOM）
#   - 原厂 NOS planner 订阅 /ODOM + occ_grid 即可端到端导航（见 docs/M20_ALIGNMENT.md §5）
#   - 不写入 /NAV_CMD、不接管原厂运动链（安全边界）
set -euo pipefail

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

CONFIG_ARG=""
ENABLE_JOINTS="true"
ENABLE_LEG_ODOM="true"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SKIP_PREFLIGHT="false"
LAUNCH_PID=""
STOPPED="false"

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "  --config PATH       覆盖 default_m20.yaml 的绝对路径" \
    "  --no-joints         不启动 joints_adapter（开发机无 drdds 时）" \
    "  --no-leg-odom       不启动 leg_wheel_odom" \
    "  --domain ID         ROS_DOMAIN_ID（默认当前或 0）" \
    "  --skip-preflight    跳过传感器话题预检查" \
    "  -h, --help          显示帮助"
}

while (($#)); do
  case "$1" in
    --config) CONFIG_ARG="${2:?missing config path}"; shift 2 ;;
    --no-joints) ENABLE_JOINTS="false"; shift ;;
    --no-leg-odom) ENABLE_LEG_ODOM="false"; shift ;;
    --domain) DOMAIN_ID="${2:?missing domain id}"; shift 2 ;;
    --skip-preflight) SKIP_PREFLIGHT="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

source_ros() {
  set +u
  if [[ -f /opt/robot/scripts/setup_ros2.sh ]]; then
    source /opt/robot/scripts/setup_ros2.sh
  elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    source /opt/ros/foxy/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
  else
    printf 'ROS 2 setup was not found.\n' >&2
    exit 1
  fi
  set -u
}

source_ros
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  printf 'Missing %s/install/setup.bash - 先在 AOS 构建:\n' "${WORKSPACE}" >&2
  printf '  source /opt/robot/scripts/setup_ros2.sh && cd %s && \\\n' "${WORKSPACE}" >&2
  printf '  colcon build --packages-select lightning leg_wheel_odom m20_joints_adapter\n' >&2
  exit 1
fi
set +u
source "${WORKSPACE}/install/setup.bash"
set -u

if pgrep -f "run_loc_online" >/dev/null 2>&1; then
  printf 'A run_loc_online process is already running. Stop it first.\n' >&2
  pgrep -af "run_loc_online" >&2 || true
  exit 1
fi

if [[ "${SKIP_PREFLIGHT}" != "true" ]]; then
  printf 'Preflight sensor topics: /LIDAR/POINTS, /IMU ...\n'
  for topic in /LIDAR/POINTS /IMU; do
    if ! ROS2CLI_NO_DAEMON=1 ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      printf '  WARN: %s not on the DDS graph yet\n' "${topic}" >&2
    fi
  done
fi

cleanup() {
  if [[ "${STOPPED}" == "true" ]]; then
    return
  fi
  STOPPED="true"
  trap '' HUP INT TERM
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  printf 'Localization session finished.\n'
}
trap cleanup EXIT HUP INT TERM

LAUNCH_ARGS=(enable_joints_adapter:="${ENABLE_JOINTS}" enable_leg_odom:="${ENABLE_LEG_ODOM}")
if [[ -n "${CONFIG_ARG}" ]]; then
  LAUNCH_ARGS+=(config:="${CONFIG_ARG}")
fi

printf 'Starting M20 localization (domain=%s) ...\n' "${DOMAIN_ID}"
setsid ros2 launch lightning m20_loc.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

wait "${LAUNCH_PID}"