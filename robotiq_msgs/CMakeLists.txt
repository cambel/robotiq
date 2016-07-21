cmake_minimum_required(VERSION 2.8.3)
project(robotiq_msgs)

find_package(catkin REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  control_msgs
  message_generation 
)

add_message_files(
  FILES 
  CModelCommand.msg 
  CModelStatus.msg
)

add_action_files(
  FILES
  CModelCommand.action
)

generate_messages(DEPENDENCIES actionlib_msgs control_msgs)

catkin_package(CATKIN_DEPENDS 
  actionlib 
  actionlib_msgs 
  control_msgs 
  message_runtime
)
