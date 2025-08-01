cmake_minimum_required(VERSION 3.5)
project(xsens_mti_ros2_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ROS dependencies
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
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(diagnostic_msgs REQUIRED)

# Check ROS distro
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
  set(ROS_DISTRO "unknown")
endif()

message(STATUS "Detected ROS_DISTRO: ${ROS_DISTRO}")

################################################
## Declare ROS messages, services and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XsStatusWord.msg"
  "msg/ShipMotion.msg"
  "msg/GnssPvt.msg"
  "msg/GnssSatInfo.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

# Build the bundled Xsens SDK (xspublic) via its own Makefile. We use add_custom_target instead of
# ExternalProject_Add because the latter's build cache went stale under colcon+ccache, leaving artifacts
# unchanged after source edits. env --unset=MAKEFLAGS prevents inherited colcon -j flags from corrupting
# the sub-make.
add_custom_target(xspublic ALL
  COMMAND env --unset=MAKEFLAGS ${CMAKE_MAKE_PROGRAM} -C ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
  COMMENT "Building Xsens SDK libraries using Makefile"
)

# Tell linker where to find the static libraries built by the Makefile
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscontroller
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscommon
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xstypes
)

## Specify additional locations of header files
include_directories(
  ${PROJECT_SOURCE_DIR}/lib/xspublic
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(xsens_mti_node
  src/main.cpp
  src/xdainterface.cpp
  src/xdacallback.cpp
  src/xsens_log_handler.cpp
  src/ntrip_util.cpp
  src/xsens_time_handler.cpp
  src/high_rate_interpolator.cpp
)

# Make sure xsens_mti_node depends on the xspublic build
add_dependencies(xsens_mti_node xspublic ${PROJECT_NAME})

if(TARGET rclcpp::rclcpp)
  # Rolling and newer: use modern CMake targets
  target_link_libraries(xsens_mti_node
    ${diagnostic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${mavros_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${nmea_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
    tf2::tf2
    tf2_ros::static_transform_broadcaster_node
    tf2_ros::tf2_ros
  )
else()
  # Humble/Jazzy: use ament_target_dependencies
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
    diagnostic_msgs
  )
endif()

# Handle typesupport linking differently based on ROS distro
if(${ROS_DISTRO} STREQUAL "foxy")
  target_link_libraries(xsens_mti_node
    ${CMAKE_BINARY_DIR}/libxsens_mti_ros2_driver__rosidl_typesupport_cpp.so
  )
  add_dependencies(xsens_mti_node
    ${PROJECT_NAME}__rosidl_typesupport_cpp
  )
  target_include_directories(xsens_mti_node PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
else()
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(xsens_mti_node "${cpp_typesupport_target}")
endif()

# Link Xsens SDK libs + system libs
target_link_libraries(xsens_mti_node
  xscontroller
  xscommon
  xstypes
  pthread
  dl
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_imu_publisher
    test/test_imu_publisher.cpp
  )
  add_dependencies(test_imu_publisher xspublic ${PROJECT_NAME})
  target_include_directories(test_imu_publisher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
  if(TARGET rclcpp::rclcpp)
    target_link_libraries(test_imu_publisher
      ${geometry_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
    )
  else()
    ament_target_dependencies(test_imu_publisher
      rclcpp
      sensor_msgs
      geometry_msgs
    )
  endif()
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_imu_publisher
    "${cpp_typesupport_target}"
    xstypes
    pthread
    dl
  )
endif()

#############
## Install ##
#############

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
  rosidl_default_runtime
  Eigen3
  diagnostic_msgs
)

ament_export_include_directories(include)

ament_package()
