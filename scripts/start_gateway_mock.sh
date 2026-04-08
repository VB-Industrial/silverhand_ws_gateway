#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${ROS_WS:-$(cd "${REPO_DIR}/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROS_WS}/install/setup.bash"

exec ros2 run silverhand_arm_ws_gateway gateway \
  --domain arm \
  --mode mock \
  --host "${SILVERHAND_WS_HOST:-0.0.0.0}" \
  --port "${SILVERHAND_WS_PORT:-8765}" \
  --log-level "${SILVERHAND_WS_LOG_LEVEL:-INFO}"
