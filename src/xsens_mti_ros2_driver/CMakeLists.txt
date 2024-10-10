#Set minimum required CMake version and project name.
cmake_minimum_required(VERSION 3.5)
project(xsens_mti_ros2_driver)


# Set C++ standard, default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

link_directories(lib/xspublic/xscontroller lib/xspublic/xscommon lib/xspublic/xstypes)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XsStatusWord.msg"
  DEPENDENCIES std_msgs
)


ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########


# Include ExternalProject module
include(ExternalProject)

# Define the external project which is make the xspublic
ExternalProject_Add(
  xspublic
  PREFIX ${CMAKE_BINARY_DIR}/xspublic
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/lib/xspublic
  CONFIGURE_COMMAND ""  # No configure step
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1  # Build in the source directory
  INSTALL_COMMAND ""  # No install step
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
)


add_executable(
	xsens_mti_node
	src/main.cpp
	src/xdainterface.cpp
	src/xdacallback.cpp
	src/xsens_log_handler.cpp
	src/ntrip_util.cpp
	src/xsens_time_handler.cpp
)

# Make sure xsens_mti_node depends on the xspublic build
add_dependencies(xsens_mti_node xspublic ${PROJECT_NAME})


# target_include_directories(xsens_mti_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic>
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp> 
#   $<INSTALL_INTERFACE:include>)

ament_target_dependencies(xsens_mti_node 
	rclcpp
	tf2
	tf2_ros
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	nmea_msgs
	mavros_msgs
	rosidl_default_runtime
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(xsens_mti_node "${cpp_typesupport_target}")


target_link_libraries(
	xsens_mti_node
	xscontroller
	xscommon
	xstypes
	pthread
	dl
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS xsens_mti_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch param rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY headers/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)



#############
## Export  ##
#############
#This command is used to specify the dependencies that other packages need to know about 
#when they depend on this package. 
#This means if another package declares a build or run dependency on this package, 
#the dependencies listed inside ament_export_dependencies will be made available to that dependent package.
ament_export_dependencies(
	rclcpp
	tf2
	tf2_ros
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	nmea_msgs
	mavros_msgs
	rosidl_default_runtime #the default runtime for ROS 2 interface (message, service, action) generation.
)

ament_export_include_directories(
	include
)


ament_package()
