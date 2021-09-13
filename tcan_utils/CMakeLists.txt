cmake_minimum_required(VERSION 3.5)
project(tcan_utils)

find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########
# NONE

#############
## Install ##
#############
install(PROGRAMS bash/vcan.sh bash/canusb.sh python/filter_calculator.py
  python/parse_candump.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
