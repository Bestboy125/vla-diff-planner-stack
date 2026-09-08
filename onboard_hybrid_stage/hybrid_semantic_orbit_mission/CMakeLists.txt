cmake_minimum_required(VERSION 3.0.2)
project(hybrid_semantic_orbit_mission)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  atomic_skill_executor
  mavros_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/hybrid_semantic_orbit_node.py
  scripts/hybrid_orbit_geometry_self_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/hybrid_orbit_geometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
