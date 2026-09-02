#!/usr/bin/env bash
# M20 Pro 建图一键启动（对齐参考 §11 接入方式；仿 m20_orignal start_mapping.sh 模式）
#
# 在 AOS(192.168.101.36) 板上运行：source setup_ros2.sh(Foxy/域0) -> 启动
# joints_adapter + leg_wheel_odom + run_slam_online -> 就绪门禁 -> Ctrl+C 自动
# 保存地图并清理进程。
#
# 用法:
#   ./scripts/start_slam.sh [--map-id NAME] [--config PATH] [--no-joints] [--no-leg-odom] [--domain ID]
#
# 说明:
#   - 默认输出 /lio_pose（system.lio_pose_topic 可改隔离话题，如 /m20_slam/pose）
#   - 不写入 /NAV_CMD、不接管原厂定位/导航链（安全边界，见 docs/M20_ALIGNMENT.md）
set -euo pipefail

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

MAP_ID="new_map"
CONFIG_ARG=""
ENABLE_JOINTS="true"
ENABLE_LEG_ODOM="true"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SKIP_PREFLIGHT="false"
LAUNCH_PID=""
SAVED_MAP="false"

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "  --map-id NAME       地图名（默认 new_map，产物在 data/<NAME>/）" \
    "  --config PATH       覆盖 default_m20.yaml 的绝对路径" \
    "  --no-joints         不启动 joints_adapter（开发机无 drdds 时）" \
    "  --no-leg-odom       不启动 leg_wheel_odom" \
    "  --domain ID         ROS_DOMAIN_ID（默认当前或 0）" \
    "  --skip-preflight    跳过传感器话题预检查" \
    "  -h, --help          显示帮助"
}

while (($#)); do
  case "$1" in
    --map-id) MAP_ID="${2:?missing map id}"; shift 2 ;;
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

if [[ ! -d "${WORKSPACE}/src" ]] && [[ ! -d "${WORKSPACE}/lightning-lm" ]]; then
  printf 'Workspace layout not recognized: %s\n' "${WORKSPACE}" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  printf 'Missing %s/install/setup.bash - 先在 AOS 构建:\n' "${WORKSPACE}" >&2
  printf '  source /opt/robot/scripts/setup_ros2.sh && cd %s && \\\n' "${WORKSPACE}" >&2
  printf '  colcon build --packages-select lightning leg_wheel_odom m20_joints_adapter\n' >&2
  exit 1
fi
set +u
source "${WORKSPACE}/install/setup.bash"
set -u

# 重复进程保护
if pgrep -f "run_slam_online" >/dev/null 2>&1; then
  printf 'A run_slam_online process is already running. Stop it before starting another mapper.\n' >&2
  pgrep -af "run_slam_online" >&2 || true
  exit 1
fi

# 传感器话题预检查（参考文档 §10：/LIDAR/POINTS 当前可能无发布；仅警告不阻断，
# launch 就绪门禁 60s 内无话题会自动 Shutdown 整条链路）
if [[ "${SKIP_PREFLIGHT}" != "true" ]]; then
  printf 'Preflight sensor topics: /LIDAR/POINTS, /IMU ...\n'
  for topic in /LIDAR/POINTS /IMU; do
    if ! ROS2CLI_NO_DAEMON=1 ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      printf '  WARN: %s not on the DDS graph yet (若已上电请等待雷达/IMU 链路建立)\n' "${topic}" >&2
    fi
  done
fi

cleanup() {
  if [[ "${SAVED_MAP}" == "true" ]]; then
    return
  fi
  SAVED_MAP="true"
  trap '' HUP INT TERM
  printf '\nSaving map to data/%s ...\n' "${MAP_ID}"
  timeout 180 ros2 service call /lightning/save_map lightning/srv/SaveMap \
    "{map_id: ${MAP_ID}}" >/dev/null 2>&1 || \
    printf '  save_map call failed (mapper may already be down)\n' >&2
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  printf 'Mapping session finished.\n'
}
trap cleanup EXIT HUP INT TERM

LAUNCH_ARGS=(enable_joints_adapter:="${ENABLE_JOINTS}" enable_leg_odom:="${ENABLE_LEG_ODOM}")
if [[ -n "${CONFIG_ARG}" ]]; then
  LAUNCH_ARGS+=(config:="${CONFIG_ARG}")
fi

printf 'Starting M20 mapping (map_id=%s, domain=%s) ...\n' "${MAP_ID}" "${DOMAIN_ID}"
setsid ros2 launch lightning m20_slam.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

wait "${LAUNCH_PID}"