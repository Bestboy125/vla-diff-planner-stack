#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_ROOT="${ISAAC_ROOT:-/opt/isaac-sim-6.0.1}"
ISAAC_DATA_ROOT="${ISAAC_DATA_ROOT:-/opt/isaac-sim-data}"
GPU_ID="${GPU_ID:-0}"
OUTPUT_DIR="${OUTPUT_DIR:-${ISAAC_DATA_ROOT}/artifacts/rgbd}"

export ISAAC_ROOT
"${SCRIPT_DIR}/preflight.sh"

mkdir -p "${OUTPUT_DIR}" "${ISAAC_DATA_ROOT}/cache/xdg" "${ISAAC_DATA_ROOT}/logs"
unset CONDA_PREFIX CONDA_DEFAULT_ENV
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
export CUDA_VISIBLE_DEVICES="${GPU_ID}"
export VK_ICD_FILENAMES="${NVIDIA_ICD:-/etc/vulkan/icd.d/nvidia_icd.json}"
export XDG_CACHE_HOME="${ISAAC_DATA_ROOT}/cache/xdg"
export OMNI_KIT_ALLOW_ROOT="${OMNI_KIT_ALLOW_ROOT:-1}"

log_file="${ISAAC_DATA_ROOT}/logs/rgbd_$(date +%Y%m%d_%H%M%S).log"
echo "[INFO] log: ${log_file}"
echo "[INFO] artifacts: ${OUTPUT_DIR}"

timeout 300s "${ISAAC_ROOT}/python.sh" "${SCRIPT_DIR}/smoke_rgbd_pose.py" \
  --output-dir "${OUTPUT_DIR}" 2>&1 | tee "${log_file}"

for artifact in rgb.png depth.npy depth_preview.png metadata.json; do
  test -s "${OUTPUT_DIR}/${artifact}"
done
echo "[PASS] RGB-D and camera-pose artifacts generated"
