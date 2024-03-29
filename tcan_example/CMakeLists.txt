cmake_minimum_required(VERSION 3.5)
project(tcan_example)

find_package(catkin REQUIRED
  COMPONENTS
    tcan
    tcan_can
    tcan_ip
    tcan_usb
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    tcan
    tcan_can
    tcan_ip
    tcan_usb
)

###########
## Build ##
###########
add_definitions(-fPIC -Wall -Wextra -Wpedantic -Werror)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_example_node
  src/CanDeviceExample.cpp
  src/CanManagerExample.cpp
  src/can_example_node.cpp
)
target_link_libraries(can_example_node
  ${catkin_LIBRARIES}
)

add_executable(usb_example_node
  src/XbeeUsb.cpp
  src/usb_example_node.cpp
)
target_link_libraries(usb_example_node
  ${catkin_LIBRARIES}
)

add_executable(tcp_example_node
  src/TcpConnection.cpp
  src/tcp_example_node.cpp
)
target_link_libraries(tcp_example_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


install(TARGETS can_example_node usb_example_node tcp_example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

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
