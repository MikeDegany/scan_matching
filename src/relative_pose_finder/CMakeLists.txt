cmake_minimum_required(VERSION 3.8)
project(relative_pose_finder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system filesystem thread)
find_package(TBB REQUIRED NO_CMAKE_PACKAGE_REGISTRY)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(OpenCV REQUIRED)

include_directories(include
${Boost_INCLUDE_DIRS}
${Python3_INCLUDE_DIRS}
# ${karto_sdk_INCLUDE_DIRS}
${TBB_INCLUDE_DIRS}
include_directories(${OpenCV_INCLUDE_DIRS})
)

add_executable(relative_pose_finder
  src/relative_pose_finder.cpp
  src/scan_matcher.cpp  
  src/main.cpp)

# Link libraries
target_link_libraries(relative_pose_finder
  ${Boost_LIBRARIES}
  # kartoSlamToolbox
  # ${CMAKE_CURRENT_SOURCE_DIR}/lib/karto_sdk
  # karto_sdk  # Link karto_sdk library
  # message("karto_sdk::karto_sdk: ${karto_sdk::karto_sdk}")
  /opt/ros/humble/lib/libkartoSlamToolbox.so
  TBB::tbb
  ${EIGEN3_LIBRARIES}
  ${Python3_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
# Add to target_include_directories
target_include_directories(relative_pose_finder PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(relative_pose_finder
  rclcpp
  sensor_msgs
  geometry_msgs
  slam_toolbox
  tf2
  tf2_ros)

install(TARGETS
  relative_pose_finder
  DESTINATION lib/${PROJECT_NAME})

ament_package()
