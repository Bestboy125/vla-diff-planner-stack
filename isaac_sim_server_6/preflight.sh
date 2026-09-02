#!/usr/bin/env bash
set -Eeuo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-/opt/isaac-sim-6.0.1}"
NVIDIA_ICD="${NVIDIA_ICD:-/etc/vulkan/icd.d/nvidia_icd.json}"

fail() {
  echo "[FAIL] $*" >&2
  exit 1
}

pass() {
  echo "[PASS] $*"
}

command -v nvidia-smi >/dev/null || fail "nvidia-smi is unavailable"
command -v vulkaninfo >/dev/null || fail "vulkaninfo is unavailable"
[[ -x "${ISAAC_ROOT}/python.sh" ]] || fail "Isaac Sim python.sh not found at ${ISAAC_ROOT}"
[[ -r "${NVIDIA_ICD}" ]] || fail "NVIDIA Vulkan ICD not found at ${NVIDIA_ICD}"

nvidia-smi --query-gpu=index,name,driver_version,memory.total,memory.used \
  --format=csv,noheader
pass "NVIDIA compute device is visible"

if [[ ! -e /dev/nvidia-modeset ]]; then
  fail "/dev/nvidia-modeset is missing; the outer container has not granted NVIDIA graphics capability"
fi

python3 - <<'PY' || fail "/dev/nvidia-modeset is present but denied by the devices cgroup"
import os

fd = os.open("/dev/nvidia-modeset", os.O_RDWR)
os.close(fd)
PY
pass "/dev/nvidia-modeset is accessible"

summary_file="$(mktemp)"
trap 'rm -f "${summary_file}"' EXIT
if ! env VK_ICD_FILENAMES="${NVIDIA_ICD}" vulkaninfo --summary >"${summary_file}" 2>&1; then
  sed -n '1,120p' "${summary_file}" >&2
  fail "NVIDIA Vulkan instance creation failed"
fi

grep -E 'deviceName|driverName|driverInfo|apiVersion' "${summary_file}" || true
pass "NVIDIA Vulkan is usable"
pass "Isaac Sim 6.0.1 headless preflight completed"
