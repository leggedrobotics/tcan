cmake_minimum_required(VERSION 3.5)
project(tcan_ip)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
find_package(ament_cmake REQUIRED)
find_package(message_logger REQUIRED)
find_package(tcan REQUIRED)

###########
## Build ##
###########
add_definitions(-fPIC -Wall -Werror)

include_directories(include)
add_library(${PROJECT_NAME} src/IpBusManager.cpp src/IpBus.cpp)
ament_target_dependencies(${PROJECT_NAME} message_logger tcan)

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
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
