#!/usr/bin/env bash

AIR_STACK_ROOT="${AIR_STACK_ROOT:-/home/airstack/AirStack}"
AIR_STACK_DOMAIN_ID="${AIR_STACK_DOMAIN_ID:-42}"
AIR_STACK_DISCOVERY_PORT="${AIR_STACK_DISCOVERY_PORT:-11811}"
AIR_STACK_WSL_IP="$(hostname -I | awk '{print $1}')"

if [[ -z "${AIR_STACK_WSL_IP}" ]]; then
  echo "ERROR: unable to determine the AirStack WSL IPv4 address" >&2
  return 1 2>/dev/null || exit 1
fi

source /opt/ros/humble/setup.bash
source "${AIR_STACK_ROOT}/.native_ws/common/install/setup.bash"
source "${AIR_STACK_ROOT}/.native_ws/robot/install/setup.bash"
source "${AIR_STACK_ROOT}/.native_ws/gcs/install/setup.bash"

export ROS_DISTRO=humble
export ROS_DOMAIN_ID="${AIR_STACK_DOMAIN_ID}"
export ROBOT_NAME="${ROBOT_NAME:-robot_${AIR_STACK_DOMAIN_ID}}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="${AIR_STACK_WSL_IP}:${AIR_STACK_DISCOVERY_PORT}"
export PATH="${HOME}/.local/bin:${PATH}"

unset AIR_STACK_WSL_IP
