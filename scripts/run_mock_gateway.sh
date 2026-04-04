#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/r/silver_ws}"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8765}"
LOG_LEVEL="${LOG_LEVEL:-INFO}"

cd "${WORKSPACE_ROOT}"

if [[ ! -d install/silverhand_arm_ws_gateway ]]; then
  echo "Building silverhand_arm_ws_gateway..."
  colcon build --packages-select silverhand_arm_ws_gateway
fi

exec env \
  PYTHONPATH="${WORKSPACE_ROOT}/install/silverhand_arm_ws_gateway/lib/python3.12/site-packages${PYTHONPATH:+:${PYTHONPATH}}" \
  "${WORKSPACE_ROOT}/install/silverhand_arm_ws_gateway/lib/silverhand_arm_ws_gateway/gateway" \
  --mode mock \
  --host "${HOST}" \
  --port "${PORT}" \
  --log-level "${LOG_LEVEL}"
