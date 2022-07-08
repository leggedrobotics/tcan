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

find_package(rosidl_default_generators REQUIRED)

add_library(${PROJECT_NAME} src/tcan_bridge.cpp)
target_include_directories(tcan_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  tcan_bridge
  SYSTEM
  "rclcpp"
  "tcan_bridge_msgs"
  "tcan_can"
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS tcan_bridge
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(bidirectional_bridge src/bidirectional_bridge.cpp)
target_include_directories(bidirectional_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bidirectional_bridge tcan_bridge)
install(TARGETS bidirectional_bridge
  DESTINATION lib/${PROJECT_NAME})

add_executable(can_to_ros_bridge src/can_to_ros_bridge.cpp)
target_include_directories(can_to_ros_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(can_to_ros_bridge tcan_bridge)
install(TARGETS can_to_ros_bridge
  DESTINATION lib/${PROJECT_NAME})

add_executable(ros_to_can_bridge src/ros_to_can_bridge.cpp)
target_include_directories(ros_to_can_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ros_to_can_bridge tcan_bridge)
install(TARGETS ros_to_can_bridge
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
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
