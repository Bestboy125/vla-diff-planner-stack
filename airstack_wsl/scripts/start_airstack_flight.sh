#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/airstack_sim_env.sh"
set -u

STATE_DIR="${HOME}/.local/state/airstack-pegasus"
LOG_FILE="${STATE_DIR}/flight.log"
# Run the version-controlled integration launch directly from the shared E:
# workspace.  The old copied file under /home/airstack/AirStack can otherwise
# silently lag behind bridge and adapter changes.
LAUNCH_FILE="${AIRSTACK_FLIGHT_LAUNCH:-${SCRIPT_DIR}/../launch/airstack_flight_native.launch.py}"
mkdir -p "${STATE_DIR}"

if [[ ! -r "${LAUNCH_FILE}" ]]; then
  echo "ERROR: native flight launch file is missing: ${LAUNCH_FILE}" >&2
  exit 10
fi
if pgrep -f "ros2 launch ${LAUNCH_FILE}" >/dev/null 2>&1; then
  echo "AirStack flight stack is already running"
  exit 0
fi

nohup ros2 launch "${LAUNCH_FILE}" >"${LOG_FILE}" 2>&1 &
PID=$!
echo "${PID}" >"${STATE_DIR}/flight.pid"

for _ in $(seq 1 90); do
  if ! kill -0 "${PID}" 2>/dev/null; then
    tail -n 160 "${LOG_FILE}" >&2
    exit 20
  fi
  if pgrep -f "/trajectory_controller/lib/trajectory_controller/trajectory_controller" >/dev/null 2>&1 &&
     pgrep -f "/trajectory_library/lib/trajectory_library/fixed_trajectory_generator.py" >/dev/null 2>&1 &&
     pgrep -f "/pid_controller/lib/pid_controller/pid_controller" >/dev/null 2>&1; then
    echo "AIRSTACK_FLIGHT_READY pid=${PID} log=${LOG_FILE}"
    exit 0
  fi
  sleep 0.5
done

echo "ERROR: AirStack flight services did not appear within 45 seconds" >&2
tail -n 160 "${LOG_FILE}" >&2
exit 21
