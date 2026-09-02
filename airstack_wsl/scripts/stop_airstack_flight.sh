#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_FILE="${AIRSTACK_FLIGHT_LAUNCH:-${SCRIPT_DIR}/../launch/airstack_flight_native.launch.py}"
DEPTH_ADAPTER="${SCRIPT_DIR}/depth_to_disparity.py"

stop_pattern() {
  local pattern="$1"
  pkill -TERM -f "${pattern}" 2>/dev/null || true
}

stop_pattern '^/opt/ros/humble/lib/tf2_ros/static_transform_publisher .*camera_front'
stop_pattern '^/usr/bin/python3 /home/airstack/AirStack/.native_ws/robot/install/trajectory_library/lib/trajectory_library/fixed_trajectory_generator.py '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/takeoff_landing_planner/lib/takeoff_landing_planner/takeoff_landing_planner '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/droan_gl/lib/droan_gl/expand '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/disparity_expansion/lib/disparity_expansion/disparity_expansion '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/trajectory_controller/lib/trajectory_controller/trajectory_controller '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/pid_controller/lib/pid_controller/pid_controller '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/behavior_tree/lib/behavior_tree/behavior_tree_implementation '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/behavior_executive/lib/behavior_executive/behavior_executive '
stop_pattern "^python3 ${DEPTH_ADAPTER} "
stop_pattern "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch ${LAUNCH_FILE}$"

sleep 2

for pattern in \
  '^/opt/ros/humble/lib/tf2_ros/static_transform_publisher .*camera_front' \
  '^/usr/bin/python3 /home/airstack/AirStack/.native_ws/robot/install/trajectory_library/lib/trajectory_library/fixed_trajectory_generator.py ' \
  '^/home/airstack/AirStack/.native_ws/robot/install/takeoff_landing_planner/lib/takeoff_landing_planner/takeoff_landing_planner ' \
  '^/home/airstack/AirStack/.native_ws/robot/install/droan_gl/lib/droan_gl/expand ' \
  '^/home/airstack/AirStack/.native_ws/robot/install/disparity_expansion/lib/disparity_expansion/disparity_expansion ' \
  '^/home/airstack/AirStack/.native_ws/robot/install/trajectory_controller/lib/trajectory_controller/trajectory_controller ' \
  '^/home/airstack/AirStack/.native_ws/robot/install/pid_controller/lib/pid_controller/pid_controller ' \
  '^/home/airstack/AirStack/.native_ws/robot/install/behavior_tree/lib/behavior_tree/behavior_tree_implementation ' \
  '^/home/airstack/AirStack/.native_ws/robot/install/behavior_executive/lib/behavior_executive/behavior_executive '
do
  pkill -KILL -f "${pattern}" 2>/dev/null || true
done
pkill -KILL -f "^python3 ${DEPTH_ADAPTER} " 2>/dev/null || true
pkill -KILL -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch ${LAUNCH_FILE}$" 2>/dev/null || true
echo "AIRSTACK_FLIGHT_STOPPED"
