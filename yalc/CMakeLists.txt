cmake_minimum_required(VERSION 2.6)
project(yalc)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yalc
#  CATKIN_DEPENDS 
  DEPENDS Boost
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
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Bus.cpp
  src/BusManager.cpp
  src/SocketBus.cpp
  src/DeviceCanOpen.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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

