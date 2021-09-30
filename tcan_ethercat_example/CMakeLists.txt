cmake_minimum_required(VERSION 3.5)
project(tcan_ethercat_example)

find_package(catkin REQUIRED COMPONENTS
  tcan_ethercat
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS tcan_ethercat
)

###########
## Build ##
###########
add_definitions(-Wall -Wextra -Wpedantic -Werror)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(elmo_twitter_example_node
  src/ElmoTwitter.cpp
  src/elmo_twitter_example_node.cpp
)
target_link_libraries(elmo_twitter_example_node
  ${catkin_LIBRARIES}
)

add_executable(anydrive_example_node
  src/Anydrive.cpp
  src/anydrive_example_node.cpp
)
target_link_libraries(anydrive_example_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


install(TARGETS elmo_twitter_example_node anydrive_example_node
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
