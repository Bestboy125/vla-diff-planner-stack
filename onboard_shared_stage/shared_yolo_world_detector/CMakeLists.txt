cmake_minimum_required(VERSION 3.0.2)
project(shared_yolo_world_detector)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

add_service_files(FILES DetectTarget.srv)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime rospy sensor_msgs std_msgs)

catkin_install_python(PROGRAMS
  scripts/shared_yolo_world_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch srv DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
