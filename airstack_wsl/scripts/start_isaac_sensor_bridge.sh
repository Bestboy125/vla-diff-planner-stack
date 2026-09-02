#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
set -u

STATE_DIR="${HOME}/.local/state/airstack-pegasus"
BRIDGE_SCRIPT="${SCRIPT_DIR}/isaac_sensor_domain_bridge.py"
WSL_IP="$(hostname -I | awk '{print $1}')"
mkdir -p "${STATE_DIR}"

if [[ ! -r "${BRIDGE_SCRIPT}" || -z "${WSL_IP}" ]]; then
  echo "ERROR: bridge script or WSL address is unavailable" >&2
  exit 10
fi

if ! pgrep -f "${BRIDGE_SCRIPT} sink" >/dev/null 2>&1; then
  env -u ROS_DISCOVERY_SERVER -u FASTRTPS_DEFAULT_PROFILES_FILE \
    ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    nohup python3 "${BRIDGE_SCRIPT}" sink >"${STATE_DIR}/sensor-bridge-sink.log" 2>&1 &
  echo "$!" >"${STATE_DIR}/sensor-bridge-sink.pid"
fi

sleep 1
if ! pgrep -f "${BRIDGE_SCRIPT} source" >/dev/null 2>&1; then
  env ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DISCOVERY_SERVER="${WSL_IP}:11811" \
    nohup python3 "${BRIDGE_SCRIPT}" source >"${STATE_DIR}/sensor-bridge-source.log" 2>&1 &
  echo "$!" >"${STATE_DIR}/sensor-bridge-source.pid"
fi

for _ in $(seq 1 40); do
  if grep -q "BRIDGE_SOURCE_CONNECTED" "${STATE_DIR}/sensor-bridge-sink.log" 2>/dev/null &&
     grep -q "BRIDGE_SOURCE_READY" "${STATE_DIR}/sensor-bridge-source.log" 2>/dev/null; then
    echo "ISAAC_SENSOR_BRIDGE_READY source_domain=42 sink_domain=43"
    exit 0
  fi
  sleep 0.25
done

echo "ERROR: Isaac sensor bridge did not become ready" >&2
tail -50 "${STATE_DIR}/sensor-bridge-sink.log" >&2 || true
tail -50 "${STATE_DIR}/sensor-bridge-source.log" >&2 || true
exit 20
