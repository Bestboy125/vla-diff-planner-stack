#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/airstack_sim_env.sh"
set -u

if ! systemctl is-active --quiet airstack-fastdds-discovery.service; then
  echo "ERROR: airstack-fastdds-discovery.service is not active." >&2
  echo "Start the Windows Isaac batch first; it starts this WSL service as root." >&2
  exit 10
fi

"${SCRIPT_DIR}/start_isaac_sensor_bridge.sh"
"${SCRIPT_DIR}/start_px4_pegasus.sh"
"${SCRIPT_DIR}/start_airstack_interface.sh"
"${SCRIPT_DIR}/start_airstack_flight.sh"

if [[ "${1:-}" == "--validate" ]]; then
  python3 "${SCRIPT_DIR}/validate_runtime_streams.py"
fi

echo "AIRSTACK_RUNTIME_READY robot=${ROBOT_NAME} domain=${ROS_DOMAIN_ID}"
