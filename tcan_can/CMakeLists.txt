cmake_minimum_required(VERSION 3.5)
project(tcan_can)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(message_logger REQUIRED)
find_package(tcan REQUIRED)

set(INCLUDE_DIRS include ${tcan_utils_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${message_logger_INCLUDE_DIRS} ${tcan_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${tcan_utils_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
  ${message_logger_LIBRARY_DIRS} ${tcan_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${tcan_utils_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${message_logger_LIBRARIES} ${tcan_LIBRARIES})

###########
## Build ##
###########
add_definitions(-fPIC -Wall -Wextra -Wpedantic -Werror)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_library(${PROJECT_NAME} src/CanBusManager.cpp src/CanBus.cpp
  src/SocketBus.cpp src/DeviceCanOpen.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBS} pthread)


##########
## Test ##
##########
# if(BUILD_TESTING)
#   ament_add_gtest(test_sae_can_msg test/j1939_can_msg.cpp)
#   ament_add_gtest(test_can_bus test/can_bus.cpp)
#   target_link_libraries(test_can_bus ${PROJECT_NAME})
# endif()

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  PATTERN ".svn"
  EXCLUDE)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(message_logger)
ament_export_dependencies(tcan)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(tcan_can)

ament_package()
