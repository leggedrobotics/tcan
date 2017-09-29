cmake_minimum_required(VERSION 2.6)
project(tcan_ethercat)

find_package(catkin QUIET)

if(catkin_FOUND)
  find_package(catkin REQUIRED COMPONENTS
    tcan
    soem
  )
else()
  find_package(tcan REQUIRED)
  find_package(soem REQUIRED)
endif()

###################################
## catkin specific configuration ##
###################################
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS tcan soem
  )
endif()

###########
## Build ##
###########
add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${soem_INCLUDE_DIRS}/soem
)
if(catkin_FOUND)
  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${soem_INCLUDE_DIRS}/soem
  )
else()
  include_directories(
    ${tcan_INCLUDE_DIRS}
    ${soem_INCLUDE_DIRS}
  )
endif()

add_library(${PROJECT_NAME}
  src/EtherCatBus.cpp
  src/EtherCatBusManager.cpp
  src/EtherCatSlave.cpp
)
if(catkin_FOUND)
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
else()
  add_dependencies(${PROJECT_NAME}
    tcan
  )

  target_link_libraries(${PROJECT_NAME}
    ${tcan_LIBRARIES}
    ${soem_LIBRARIES}
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
    ${tcan_LIBRARIES}
    ${soem_LIBRARIES}
  )

  # Register the local build in case on doesn't use "make install"
  export(PACKAGE tcan_ethercat)

  # Create variable with the library location
  set(LIB_NAME tcan_ethercat)
  set(tcan_ethercat_library $<TARGET_FILE:${LIB_NAME}>)

  # Create variable for the local build tree
  get_property(tcan_ethercat_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  # Configure config file for local build tree
  configure_file(tcan_ethercatConfig.cmake.in
    "${PROJECT_BINARY_DIR}/tcan_ethercatConfig.cmake" @ONLY
  )

### Install Build ###

  # Change the include location for the case of an install location
  set(tcan_ethercat_include_dirs
    ${CMAKE_INSTALL_PREFIX}/include
    ${tcan_INCLUDE_DIRS}
    ${soem_INCLUDE_DIRS}
    ${soem_INCLUDE_DIRS}/soem
  )

  # We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
  configure_file(tcan_ethercatConfig.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/tcan_ethercatConfig.cmake" @ONLY
  )

  install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/tcan_ethercatConfig.cmake"
    DESTINATION share/tcan_ethercat/cmake COMPONENT dev
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/tcan_ethercat/
    DESTINATION include/tcan_ethercat
  )
endif()
