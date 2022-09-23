cmake_minimum_required(VERSION 3.1.3)
project(tcan_bridge_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
