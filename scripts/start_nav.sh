#!/usr/bin/env bash
# M20 Pro 自研导航一键启动（基于 m20_orignal 移植）
# 使用前需已运行建图或定位链路获得 /ODOM + /lio_pose + occ_grid
set -euo pipefail

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

MOTION_OUTPUT="false"
OCC_GRID_BRIDGE="false"
GRID_PATH=""
CONFIG=""
LAUNCH_PID=""

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "  --motion             启用 /NAV_CMD 运动输出（需确认 DrDDS 链路可用）" \
    "  --occ-grid PATH      启动 occ_grid_bridge，将 PATH/occ_grid.pgm 发布为 /GRID_MAP" \
    "  --config PATH        覆盖 native_navigation.yaml 路径" \
    "  -h, --help           显示帮助"
}

while (($#)); do
  case "$1" in
    --motion) MOTION_OUTPUT="true"; shift ;;
    --occ-grid) OCC_GRID_BRIDGE="true"; GRID_PATH="${2:?missing grid path}"; shift 2 ;;
    --config) CONFIG="${2:?missing config path}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

source_ros() {
  set +u
  if [[ -f /opt/robot/scripts/setup_ros2.sh ]]; then
    source /opt/robot/scripts/setup_ros2.sh
  elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    source /opt/ros/foxy/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
  else
    printf 'ROS2 setup not found.\n' >&2; exit 1
  fi
  set -u
}
source_ros
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  printf 'Build first: cd %s && colcon build --packages-select m20_navigation\n' "${WORKSPACE}" >&2
  exit 1
fi
set +u; source "${WORKSPACE}/install/setup.bash"; set -u

LAUNCH_ARGS=(enable_motion_output:="${MOTION_OUTPUT}")
[[ -n "${CONFIG}" ]] && LAUNCH_ARGS+=(config:="${CONFIG}")
if [[ "${OCC_GRID_BRIDGE}" == "true" ]]; then
  LAUNCH_ARGS+=(enable_occ_grid_bridge:=true grid_path:="${GRID_PATH}")
fi

cleanup() { if [[ -n "${LAUNCH_PID}" ]]; then kill -INT -- "-${LAUNCH_PID}" 2>/dev/null; wait 2>/dev/null; fi; }
trap cleanup EXIT INT TERM

printf 'Starting navigation (motion_output=%s, occ_grid=%s) ...\n' "${MOTION_OUTPUT}" "${OCC_GRID_BRIDGE}"
setsid ros2 launch m20_navigation navigation.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!
wait "${LAUNCH_PID}"