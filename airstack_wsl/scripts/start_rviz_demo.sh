#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
source "${SCRIPT_DIR}/airstack_sim_env.sh"

exec rviz2 -d "${PROJECT_ROOT}/airstack_wsl/config/vla_droan_demo.rviz"
