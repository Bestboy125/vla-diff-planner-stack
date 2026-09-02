#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_SCRIPT="${SCRIPT_DIR}/isaac_sensor_domain_bridge.py"
pkill -TERM -f "^python3 ${BRIDGE_SCRIPT} (source|sink)$" 2>/dev/null || true
sleep 1
pkill -KILL -f "^python3 ${BRIDGE_SCRIPT} (source|sink)$" 2>/dev/null || true
echo "ISAAC_SENSOR_BRIDGE_STOPPED"
