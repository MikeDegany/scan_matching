cmake_minimum_required(VERSION 3.8)
project(scan_matcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(karto_sdk REQUIRED)

include_directories(
  include
  ${karto_sdk_INCLUDE_DIRS}
)

add_executable(scan_matcher
  src/scan_matcher_node.cpp
  src/scan_matcher.cpp)

target_link_libraries(scan_matcher
  ${karto_sdk_LIBRARIES}
)

ament_target_dependencies(scan_matcher
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  slam_toolbox
  karto_sdk
)

install(TARGETS
  scan_matcher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
