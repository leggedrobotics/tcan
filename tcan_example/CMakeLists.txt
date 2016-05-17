cmake_minimum_required(VERSION 2.8.3)
project(tcan_example)

find_package(catkin REQUIRED COMPONENTS
  tcan
  robot_utils
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tcan_example
 CATKIN_DEPENDS tcan robot_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_definitions(--std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tcan_example_node
  src/DeviceExample.cpp
  src/tcan_example_node.cpp
)

target_link_libraries(tcan_example_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


install(TARGETS tcan_example_node
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
