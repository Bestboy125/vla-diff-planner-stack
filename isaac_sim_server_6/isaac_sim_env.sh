#!/usr/bin/env bash
# Source this file before running custom Isaac Sim 6.0.1 applications.

export ISAAC_ROOT="/opt/isaac-sim-6.0.1"
export ISAAC_DATA_ROOT="/opt/isaac-sim-data"
export ISAAC_TOOLS_ROOT="/opt/isaac-sim-tools"
export NVIDIA_ICD="/etc/vulkan/icd.d/nvidia_icd.json"
export VK_ICD_FILENAMES="${NVIDIA_ICD}"
export XDG_CACHE_HOME="${ISAAC_DATA_ROOT}/cache/xdg"
export OMNI_KIT_ALLOW_ROOT=1

mkdir -p "${XDG_CACHE_HOME}" "${ISAAC_DATA_ROOT}/logs" "${ISAAC_DATA_ROOT}/artifacts"
