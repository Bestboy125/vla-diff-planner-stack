#!/usr/bin/env bash
set -euo pipefail

stop_pattern() {
  local pattern="$1"
  pkill -TERM -f "${pattern}" 2>/dev/null || true
}

stop_pattern '^/opt/ros/humble/lib/mavros/mavros_node '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/robot_interface/lib/robot_interface/robot_interface_node '
stop_pattern '^/usr/bin/python3 /home/airstack/AirStack/.native_ws/robot/install/mavros_interface/lib/mavros_interface/position_setpoint_pub.py '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/robot_interface/lib/robot_interface/odometry_conversion '
stop_pattern '^/home/airstack/AirStack/.native_ws/robot/install/drone_safety_monitor/lib/drone_safety_monitor/drone_safety_monitor '
stop_pattern '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch /home/airstack/AirStack/airstack_interface_native.launch.py$'

sleep 2

# Some native C++ nodes do not exit on the first SIGTERM when the launch
# parent has already disappeared.  Kill only the same exact executable set.
pkill -KILL -f '^/opt/ros/humble/lib/mavros/mavros_node ' 2>/dev/null || true
pkill -KILL -f '^/home/airstack/AirStack/.native_ws/robot/install/robot_interface/lib/robot_interface/robot_interface_node ' 2>/dev/null || true
pkill -KILL -f '^/usr/bin/python3 /home/airstack/AirStack/.native_ws/robot/install/mavros_interface/lib/mavros_interface/position_setpoint_pub.py ' 2>/dev/null || true
pkill -KILL -f '^/home/airstack/AirStack/.native_ws/robot/install/robot_interface/lib/robot_interface/odometry_conversion ' 2>/dev/null || true
pkill -KILL -f '^/home/airstack/AirStack/.native_ws/robot/install/drone_safety_monitor/lib/drone_safety_monitor/drone_safety_monitor ' 2>/dev/null || true
echo "AIRSTACK_INTERFACE_STOPPED"
