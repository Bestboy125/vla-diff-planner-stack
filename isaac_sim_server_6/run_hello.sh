#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_ROOT="${ISAAC_ROOT:-/opt/isaac-sim-6.0.1}"
ISAAC_DATA_ROOT="${ISAAC_DATA_ROOT:-/opt/isaac-sim-data}"
GPU_ID="${GPU_ID:-0}"

export ISAAC_ROOT
"${SCRIPT_DIR}/preflight.sh"

mkdir -p "${ISAAC_DATA_ROOT}/cache/xdg" "${ISAAC_DATA_ROOT}/logs"
unset CONDA_PREFIX CONDA_DEFAULT_ENV
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
export CUDA_VISIBLE_DEVICES="${GPU_ID}"
export VK_ICD_FILENAMES="${NVIDIA_ICD:-/etc/vulkan/icd.d/nvidia_icd.json}"
export XDG_CACHE_HOME="${ISAAC_DATA_ROOT}/cache/xdg"
export OMNI_KIT_ALLOW_ROOT="${OMNI_KIT_ALLOW_ROOT:-1}"

log_file="${ISAAC_DATA_ROOT}/logs/hello_$(date +%Y%m%d_%H%M%S).log"
echo "[INFO] log: ${log_file}"

timeout 300s "${ISAAC_ROOT}/python.sh" \
  "${ISAAC_ROOT}/standalone_examples/api/isaacsim.simulation_app/hello_world.py" \
  --no-window --/app/window/enabled=false 2>&1 | tee "${log_file}"
