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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(tcan_bridge_msgs REQUIRED)
find_package(tcan_can REQUIRED)

add_executable(bidirectional_bridge src/bidirectional_bridge.cpp)
target_include_directories(bidirectional_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  bidirectional_bridge
  SYSTEM
  "rclcpp"
  "tcan_bridge_msgs"
  "tcan_can"
)
install(TARGETS bidirectional_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(
  include
)
ament_export_libraries(
  tcan_bridge
)
ament_export_dependencies(message_logger)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tcan_bridge_msgs)
ament_export_dependencies(tcan_can)

ament_package()
