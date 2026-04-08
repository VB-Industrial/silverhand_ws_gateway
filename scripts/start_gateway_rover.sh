#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${ROS_WS:-$(cd "${REPO_DIR}/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROS_WS}/install/setup.bash"

exec ros2 run silverhand_arm_ws_gateway gateway \
  --domain rover \
  --mode "${SILVERHAND_WS_MODE:-ros}" \
  --host "${SILVERHAND_WS_HOST:-0.0.0.0}" \
  --port "${SILVERHAND_WS_PORT:-8766}" \
  --rover-cmd-vel-topic "${SILVERHAND_ROVER_CMD_VEL_TOPIC:-/rover_base_controller/cmd_vel_unstamped}" \
  --rover-odom-topic "${SILVERHAND_ROVER_ODOM_TOPIC:-/rover_base_controller/odom}" \
  --rover-battery-topic "${SILVERHAND_ROVER_BATTERY_TOPIC:-/battery_state}" \
  --rover-headlights-service "${SILVERHAND_ROVER_HEADLIGHTS_SERVICE:-/power_board/set_headlights}" \
  --log-level "${SILVERHAND_WS_LOG_LEVEL:-INFO}"
