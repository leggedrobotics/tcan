cmake_minimum_required(VERSION 2.8.3)
project(pcan_driver)

find_package(catkin REQUIRED)

set( TAR_FILE ${CMAKE_CURRENT_SOURCE_DIR}/peak-linux-driver-8.3.1.tar.gz )
set( EXTRACTED_DIR ${CMAKE_BINARY_DIR}/peak-linux-driver-8.3.1 )
set( SRC_DIR ${EXTRACTED_DIR}/lib/src )
set( DEVEL_INCLUDE_DIR ${CATKIN_DEVEL_PREFIX}/include )

file(MAKE_DIRECTORY ${DEVEL_INCLUDE_DIR})

## unpack driver source code archive to binary dir
execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar xvf ${TAR_FILE}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

## copy files to devel space
configure_file(${EXTRACTED_DIR}/lib/libpcan.h   ${DEVEL_INCLUDE_DIR}/${PROJECT_NAME}/libpcan.h      COPYONLY)
configure_file(${EXTRACTED_DIR}/lib/libpcanfd.h ${DEVEL_INCLUDE_DIR}/${PROJECT_NAME}/libpcanfd.h    COPYONLY)
configure_file(${EXTRACTED_DIR}/driver/pcan.h   ${DEVEL_INCLUDE_DIR}/${PROJECT_NAME}/pcan.h         COPYONLY)
configure_file(${EXTRACTED_DIR}/driver/pcanfd.h ${DEVEL_INCLUDE_DIR}/${PROJECT_NAME}/pcanfd.h       COPYONLY)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    ${DEVEL_INCLUDE_DIR}
    ${DEVEL_INCLUDE_DIR}/${PROJECT_NAME} # add include directory directly, because include directives in API source are not catkin style packaged based
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

add_definitions(-DPCANFD_OLD_STYLE_API -DNO_RT -fPIC -shared -Wall)

include_directories(
  ${DEVEL_INCLUDE_DIR}/${PROJECT_NAME} # add package name, because source is not catkin style packaged based
  ${EXTRACTED_DIR}/lib # for libprivate.h
)


add_library(${PROJECT_NAME}
  ${SRC_DIR}/libpcanfd.c
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${DEVEL_INCLUDE_DIR}/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

