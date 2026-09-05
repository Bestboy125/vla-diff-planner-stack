cmake_minimum_required(VERSION 3.0.2)
project(semantic_raw_stereo_localizer)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  atomic_skill_executor
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/semantic_raw_stereo_node.py
  scripts/semantic_orbit_executor_node.py
  scripts/semantic_orbit_contract_self_test.py
  scripts/raw_stereo_geometry_self_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/raw_stereo_geometry.py
  scripts/semantic_orbit_contract.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
