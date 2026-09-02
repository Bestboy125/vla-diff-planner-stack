#!/usr/bin/env bash

# Source the base native ROS environment, then decouple DDS domain 42 from the
# PX4/MAVLink vehicle identity (instance 0 -> MAVLink system 1).
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export AIR_STACK_DOMAIN_ID="${AIR_STACK_DOMAIN_ID:-43}"
export AIR_STACK_ROBOT_NAME="${AIR_STACK_ROBOT_NAME:-robot_1}"
source "${SCRIPT_DIR}/airstack_env.sh"

# Keep the full AirStack graph on local Simple Discovery. Isaac Sim remains on
# domain 42 behind the Discovery Server; the sensor bridge republishes into 43.
export ROS_DOMAIN_ID="${AIR_STACK_DOMAIN_ID}"
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE

export ROBOT_NAME="${AIR_STACK_ROBOT_NAME}"
export PX4_INSTANCE="${PX4_INSTANCE:-0}"
export TGT_SYSTEM="${TGT_SYSTEM:-1}"
export FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14580}"
export SIM_TYPE="${SIM_TYPE:-not_simple}"
export RECORD_BAGS="${RECORD_BAGS:-false}"

export ROBOT_DESCRIPTION_PACKAGE="${ROBOT_DESCRIPTION_PACKAGE:-iris_with_sensors_description}"
export ROBOT_URDF_FILE="${ROBOT_URDF_FILE:-iris_with_sensors.pegasus.robot.urdf}"
export AUTONOMY_LAUNCH_PACKAGE="${AUTONOMY_LAUNCH_PACKAGE:-autonomy_bringup}"
export AUTONOMY_LAUNCH_FILE="${AUTONOMY_LAUNCH_FILE:-autonomy.launch.xml}"
export INTERFACE_LAUNCH_PACKAGE="${INTERFACE_LAUNCH_PACKAGE:-interface_bringup}"
export INTERFACE_LAUNCH_FILE="${INTERFACE_LAUNCH_FILE:-interface.launch.xml}"
export SENSORS_LAUNCH_PACKAGE="${SENSORS_LAUNCH_PACKAGE:-sensors_bringup}"
export SENSORS_LAUNCH_FILE="${SENSORS_LAUNCH_FILE:-sensors.launch.xml}"
export PERCEPTION_LAUNCH_PACKAGE="${PERCEPTION_LAUNCH_PACKAGE:-perception_bringup}"
export PERCEPTION_LAUNCH_FILE="${PERCEPTION_LAUNCH_FILE:-perception.launch.xml}"
export LOCAL_LAUNCH_PACKAGE="${LOCAL_LAUNCH_PACKAGE:-local_bringup}"
export LOCAL_LAUNCH_FILE="${LOCAL_LAUNCH_FILE:-local.launch.xml}"
export GLOBAL_LAUNCH_PACKAGE="${GLOBAL_LAUNCH_PACKAGE:-global_bringup}"
export GLOBAL_LAUNCH_FILE="${GLOBAL_LAUNCH_FILE:-global.launch.xml}"
export BEHAVIOR_LAUNCH_PACKAGE="${BEHAVIOR_LAUNCH_PACKAGE:-behavior_bringup}"
export BEHAVIOR_LAUNCH_FILE="${BEHAVIOR_LAUNCH_FILE:-behavior.launch.xml}"
