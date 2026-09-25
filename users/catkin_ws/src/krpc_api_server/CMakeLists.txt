cmake_minimum_required(VERSION 3.0.2)
project(krpc_api_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  krpc_ib2_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs actionlib actionlib_msgs krpc_ib2_msgs
)

catkin_install_python(PROGRAMS
  scripts/cheat_sheet_loader.py
  scripts/krpc_api_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
