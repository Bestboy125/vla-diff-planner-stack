cmake_minimum_required(VERSION 3.0.2)
project(semantic_3d_localizer)
find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs quadrotor_msgs rospy visualization_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/semantic_target_node.py
  scripts/semantic_target_sim.py
  scripts/geometry_self_test.py
  scripts/yolo_world_infer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

