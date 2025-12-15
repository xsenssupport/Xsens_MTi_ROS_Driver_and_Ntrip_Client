cmake_minimum_required(VERSION 3.8)
project(ntrip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(include)

# Create the ntrip library
add_library(${PROJECT_NAME}_lib
  src/ntrip_client.cpp
  src/rtcm_parser.cpp
  src/ntrip_client_lifecycle.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  nmea_msgs
  mavros_msgs
  diagnostic_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  Boost::system
  Boost::thread
)

# Create the node executable
add_executable(ntrip src/ntrip_client_node.cpp)
add_executable(ntrip_lifecycle src/ntrip_client_lifecycle_node.cpp)

target_link_libraries(ntrip
  ${PROJECT_NAME}_lib
)

target_link_libraries(ntrip_lifecycle
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(ntrip
  rclcpp
)

ament_target_dependencies(ntrip_lifecycle
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}_lib
  ntrip
  ntrip_lifecycle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()