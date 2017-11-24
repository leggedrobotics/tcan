cmake_minimum_required(VERSION 2.6)
project(tcan_utils)

find_package(catkin REQUIRED COMPONENTS
  # NONE
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    # NONE
  LIBRARIES
    # NONE
  CATKIN_DEPENDS
    # NONE
)

###########
## Build ##
###########
# NONE

#############
## Install ##
#############
install(
  PROGRAMS
    bash/vcan.sh
    bash/canusb.sh
    python/filter_calculator.py
    python/parse_candump.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
