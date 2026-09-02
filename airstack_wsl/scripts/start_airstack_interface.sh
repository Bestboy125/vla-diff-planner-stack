#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/airstack_sim_env.sh"
set -u

STATE_DIR="${HOME}/.local/state/airstack-pegasus"
LOG_FILE="${STATE_DIR}/interface.log"
LAUNCH_FILE="${AIR_STACK_ROOT}/airstack_interface_native.launch.py"
mkdir -p "${STATE_DIR}"

if [[ ! -r "${LAUNCH_FILE}" ]]; then
  echo "ERROR: native interface launch file is missing: ${LAUNCH_FILE}" >&2
  exit 10
fi
if pgrep -f "ros2 launch ${LAUNCH_FILE}" >/dev/null 2>&1; then
  echo "AirStack interface is already running"
  exit 0
fi

nohup ros2 launch "${LAUNCH_FILE}" >"${LOG_FILE}" 2>&1 &
PID=$!
echo "${PID}" >"${STATE_DIR}/interface.pid"

for _ in $(seq 1 60); do
  if ! kill -0 "${PID}" 2>/dev/null; then
    tail -n 120 "${LOG_FILE}" >&2
    exit 20
  fi
  if pgrep -f "/mavros/mavros_node" >/dev/null 2>&1 &&
     pgrep -f "/robot_interface/lib/robot_interface/robot_interface_node" >/dev/null 2>&1 &&
     grep -Eq "Got HEARTBEAT, connected|link\[1000\] detected remote address 1\.1" "${LOG_FILE}" 2>/dev/null; then
    echo "AIRSTACK_INTERFACE_READY pid=${PID} log=${LOG_FILE}"
    exit 0
  fi
  sleep 0.5
done

echo "ERROR: AirStack interface service did not appear within 30 seconds" >&2
tail -n 120 "${LOG_FILE}" >&2
exit 21
