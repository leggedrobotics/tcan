cmake_minimum_required(VERSION 3.5)
project(tcan_can)

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
  src/CanBusManager.cpp
  src/CanBus.cpp
  src/DeviceCanOpen.cpp
  src/SocketBus.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pthread
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_can_bus test/can_bus.cpp)
    target_link_libraries(test_can_bus ${PROJECT_NAME})
endif()

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
