cmake_minimum_required(VERSION 2.6)
project(tcan_pcan)

find_package(catkin REQUIRED COMPONENTS
  tcan
  pcan_driver
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} pthread pcan
  CATKIN_DEPENDS tcan pcan_driver
)

###########
## Build ##
###########
add_definitions(-Wall )
add_definitions(-fPIC)
add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/PcanBus.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pthread
  pcan
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)