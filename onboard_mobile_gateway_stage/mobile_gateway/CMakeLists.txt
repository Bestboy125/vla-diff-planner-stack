cmake_minimum_required(VERSION 3.0.2)
project(mobile_gateway)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  atomic_skill_executor
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mobile_gateway_node.py
  scripts/gateway_smoke_client.py
  scripts/simulated_camera_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/command_protocol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
