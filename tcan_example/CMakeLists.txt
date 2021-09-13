cmake_minimum_required(VERSION 3.5)
project(tcan_example)
find_package(tcan_ip REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tcan_can REQUIRED)
find_package(tcan REQUIRED)
find_package(tcan_usb REQUIRED)

include_directories(include)

###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########

add_definitions(-fPIC -Wall -Werror)

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(can_example_node src/CanDeviceExample.cpp
  src/CanManagerExample.cpp src/can_example_node.cpp)
ament_target_dependencies(can_example_node tcan_can)

add_executable(usb_example_node src/XbeeUsb.cpp src/usb_example_node.cpp)
ament_target_dependencies(usb_example_node tcan_usb)

add_executable(tcp_example_node src/TcpConnection.cpp src/tcp_example_node.cpp)
ament_target_dependencies(tcp_example_node tcan_ip)

#############
## Install ##
#############

install(TARGETS can_example_node 
usb_example_node 
tcp_example_node
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  PATTERN ".svn"
  EXCLUDE)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tcan_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_export_dependencies(tcan_ip)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(tcan_can)
ament_export_dependencies(tcan)
ament_export_dependencies(tcan_usb)
ament_export_include_directories(include)

ament_package()
