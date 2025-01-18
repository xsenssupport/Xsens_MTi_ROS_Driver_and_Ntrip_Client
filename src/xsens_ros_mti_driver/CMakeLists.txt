cmake_minimum_required(VERSION 2.8.3)
project(xsens_mti_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  nmea_msgs
  sensor_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
  XsStatusWord.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xsens_mti_driver
  CATKIN_DEPENDS
  message_runtime
  roscpp
  tf2
  tf2_ros
  nmea_msgs
  sensor_msgs
  std_msgs
  geometry_msgs

#  DEPENDS system_lib
)

###########
## Build ##
###########

# Include ExternalProject module
include(ExternalProject)

# Define the external project which builds the xspublic
ExternalProject_Add(
  xspublic
  PREFIX ${CMAKE_BINARY_DIR}/xspublic
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
  CONFIGURE_COMMAND ""  # No configure step
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1  # Build in the source directory
  INSTALL_COMMAND ""  # No install step
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  lib/xspublic
  ${PROJECT_SOURCE_DIR}/include/ #to include headers in the include directory
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib/xspublic/xscontroller lib/xspublic/xscommon lib/xspublic/xstypes)

add_executable(
  xsens_mti_node
  src/main.cpp
  src/xdainterface.cpp
  src/xdacallback.cpp
  src/xsens_log_handler.cpp
  src/ntrip_util.cpp
  src/xsens_time_handler.cpp
)

# Make xsens_mti_node depend on xspublic and message generation
add_dependencies(xsens_mti_node 
  xspublic
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  xsens_mti_node
  xscontroller
  xscommon
  xstypes
  pthread
  dl
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS xsens_mti_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE)

