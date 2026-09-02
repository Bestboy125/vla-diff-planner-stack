#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_BIN="/home/airstack/PX4-Autopilot/build/px4_sitl_default/bin/px4"

"${SCRIPT_DIR}/stop_airstack_flight.sh"
"${SCRIPT_DIR}/stop_airstack_interface.sh"
"${SCRIPT_DIR}/stop_isaac_sensor_bridge.sh"

mapfile -t px4_pids < <(pgrep -f "^${PX4_BIN} .* -i 0 -d$" || true)
if ((${#px4_pids[@]})); then
  kill -TERM "${px4_pids[@]}" 2>/dev/null || true
  sleep 2
  for px4_pid in "${px4_pids[@]}"; do
    kill -KILL "${px4_pid}" 2>/dev/null || true
  done
fi

echo "AIRSTACK_RUNTIME_STOPPED (Windows Isaac Sim left running)"
