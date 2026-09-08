cmake_minimum_required(VERSION 3.0.2)
project(semantic_scan_orbit_mission)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  atomic_skill_executor
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/semantic_scan_orbit_node.py
  scripts/scan_route_geometry_self_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/scan_route_geometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
