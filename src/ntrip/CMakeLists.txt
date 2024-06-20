cmake_minimum_required(VERSION 3.5)
project(ntrip)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(ntrip src/ntrip_util.cpp src/ntrip_client.cpp src/client.cpp)
ament_target_dependencies(ntrip rclcpp mavros_msgs nmea_msgs std_msgs geometry_msgs)

install(TARGETS
  ntrip
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

