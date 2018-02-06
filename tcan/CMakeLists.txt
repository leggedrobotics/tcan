cmake_minimum_required(VERSION 2.6)
project(tcan)

find_package(catkin QUIET)

if(catkin_FOUND)
  find_package(catkin REQUIRED COMPONENTS
    message_logger
  )
else()
  find_package(message_logger REQUIRED)
endif()

###################################
## catkin specific configuration ##
###################################
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} pthread
    CATKIN_DEPENDS message_logger
  )
endif()

###########
## Build ##
###########
add_definitions(-std=c++11 -fPIC -Wall -Werror)

include_directories(
  include
)
if(catkin_FOUND)
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
else()
  include_directories(
    ${message_logger_INCLUDE_DIRS}
  )
endif()

add_library(${PROJECT_NAME}
  src/helper_functions.cpp
)
target_link_libraries(${PROJECT_NAME}
  pthread
)
if(catkin_FOUND)
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
else()
  target_link_libraries(${PROJECT_NAME}
    ${message_logger_LIBRARIES}
  )

  add_dependencies(${PROJECT_NAME}
    message_logger
  )
endif()

#############
## Install ##
#############
if(catkin_FOUND)
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

### Local Build ###

if(NOT catkin_FOUND)
  set(external_libs
    ${message_logger_LIBRARIES}
  )

  # Register the local build in case on doesn't use "make install"
  export(PACKAGE tcan)

  # Create variable with the library location
  set(LIB_NAME tcan)
  set(tcan_library $<TARGET_FILE:${LIB_NAME}>)

  # Create variable for the local build tree
  get_property(tcan_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  # Configure config file for local build tree
  configure_file(tcanConfig.cmake.in
    "${PROJECT_BINARY_DIR}/tcanConfig.cmake" @ONLY
  )

### Install Build ###

  # Change the include location for the case of an install location
  set(tcan_include_dirs
    ${CMAKE_INSTALL_PREFIX}/include
    ${message_logger_INCLUDE_DIRS}
  )

  # We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
  configure_file(tcanConfig.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/tcanConfig.cmake" @ONLY
  )

  install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/tcanConfig.cmake"
    DESTINATION share/tcan/cmake COMPONENT dev
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/tcan/
    DESTINATION include/tcan
  )
endif()
