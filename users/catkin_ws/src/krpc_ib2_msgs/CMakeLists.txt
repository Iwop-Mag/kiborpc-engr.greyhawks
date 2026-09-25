cmake_minimum_required(VERSION 3.0.2)
project(krpc_ib2_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GsData.msg
)

add_service_files(
  FILES
  CheckPoints.srv
  MissionStart.srv
  MissionFinished.srv
)

add_action_files(
  FILES
  CheckpointPassed.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs
)
