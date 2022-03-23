cmake_minimum_required(VERSION 3.1.3)
project(tcan_can_j1939)

set(CATKIN_BUILD_DEPENDENCIES 
  tcan_can
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_BUILD_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_BUILD_DEPENDENCIES}
  # DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wpedantic -Wextra -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json required by clang tooling

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/DeviceJ1939.cpp
  src/Devices.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

#############
## Testing ##
#############
# if(CATKIN_ENABLE_TESTING)
#   file(GLOB test_srcs ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)

#   catkin_add_gtest(test_${PROJECT_NAME}
#     ${test_srcs}
#     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
#     )

#   if(TARGET test_${PROJECT_NAME})
#     target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
#   endif()
# endif()

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

#install(DIRECTORY
#  param
#  launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )
