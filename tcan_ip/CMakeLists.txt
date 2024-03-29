cmake_minimum_required(VERSION 3.5)
project(tcan_ip)

find_package(catkin REQUIRED COMPONENTS
  message_logger
  tcan
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} pthread
  CATKIN_DEPENDS 
    message_logger
    tcan
)

###########
## Build ##
###########
add_definitions(-fPIC -Wall -Wextra -Wpedantic -Werror)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/IpBusManager.cpp
  src/IpBus.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pthread
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

