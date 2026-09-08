cmake_minimum_required(VERSION 3.0.2)
project(monocular_baseline_semantic_orbit)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  atomic_skill_executor
  geometry_msgs
  mavros_msgs
  message_filters
  nav_msgs
  rospy
  sensor_msgs
  shared_yolo_world_detector
  std_msgs
  tf2_ros
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/monocular_semantic_orbit_node.py
  scripts/monocular_orbit_geometry_self_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/monocular_orbit_geometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES THIRD_PARTY_NOTICES.md REFERENCE_VALIDATION.md README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
