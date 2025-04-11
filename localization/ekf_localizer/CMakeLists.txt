cmake_minimum_required(VERSION 3.5)
project(ekf_localizer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(ekf_localizer_node
  src/ekf_localizer.cpp
)

# Link libraries
ament_target_dependencies(ekf_localizer_node
  rclcpp
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  tf2_ros
  Eigen3
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  ekf_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install params files
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  tf2_ros
  Eigen3
  tf2_geometry_msgs
)

# Install include files
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()