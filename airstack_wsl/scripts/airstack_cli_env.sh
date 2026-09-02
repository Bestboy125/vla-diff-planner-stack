#!/usr/bin/env bash

# AirStack CLI uses the same local discovery domain as the runtime graph.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/airstack_sim_env.sh"
