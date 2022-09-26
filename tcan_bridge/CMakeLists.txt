cmake_minimum_required(VERSION 3.5)
project(tcan_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CATKIN_PACKAGE_DEPENDENCIES
  tcan_bridge_msgs
  tcan_can
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
) 

add_executable(bidirectional_bridge src/bidirectional_bridge.cpp)
target_link_libraries(bidirectional_bridge ${catkin_LIBRARIES})

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

