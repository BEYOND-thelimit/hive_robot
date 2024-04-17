cmake_minimum_required(VERSION 3.8)
project(hive_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(csm REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)

add_executable(laser_scan_matcher_node src/laser_scan_matcher.cpp)
ament_target_dependencies(laser_scan_matcher_node
  rclcpp
  sensor_msgs
  geometry_msgs
  csm
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pose_estimation src/pose_estimation.cpp)
ament_target_dependencies(pose_estimation
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(laser_scan_matcher_node
  PRIVATE
  "include"
)

# node
install(TARGETS laser_scan_matcher_node pose_estimation
  DESTINATION lib/${PROJECT_NAME}
)

# Folder
install(DIRECTORY
  include
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()