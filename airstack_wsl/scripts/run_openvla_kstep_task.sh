#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/airstack_sim_env.sh"
set -u

K="${OPENVLA_INFERENCE_EVERY_K:-3}"
INSTRUCTION="${OPENVLA_TASK_INSTRUCTION:-Fly forward and avoid the utility pole}"
OUTPUT="${OPENVLA_TASK_OUTPUT:-/mnt/e/embodied_agent/vla_planner_project/artifacts/openvla_kstep_droan_closed_loop.json}"

exec python3 "${SCRIPT_DIR}/execute_openvla_kstep_droan_task.py" \
  --inference-every-k "${K}" \
  --instruction "${INSTRUCTION}" \
  --output "${OUTPUT}" \
  "$@"
