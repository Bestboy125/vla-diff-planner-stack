#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/airstack_sim_env.sh"

STATE_DIR="${HOME}/.local/state/airstack-pegasus"
PID_FILE="${STATE_DIR}/droan-demo.pid"
LOG_FILE="${STATE_DIR}/droan-demo.log"
mkdir -p "${STATE_DIR}"

if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  kill "$(cat "${PID_FILE}")" || true
  sleep 1
fi

nohup env \
  DROAN_PLAN_ALTITUDE="${DROAN_PLAN_ALTITUDE:-0.5}" \
  DROAN_PLAN_LENGTH="${DROAN_PLAN_LENGTH:-3.5}" \
  bash -lc "while true; do source '${SCRIPT_DIR}/airstack_sim_env.sh'; python3 '${SCRIPT_DIR}/validate_droan_avoidance.py'; sleep 1; done" \
  >"${LOG_FILE}" 2>&1 &

echo $! >"${PID_FILE}"
echo "DROAN_DEMO_READY pid=$(cat "${PID_FILE}") log=${LOG_FILE}"
