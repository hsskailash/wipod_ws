cmake_minimum_required(VERSION 3.8)
project(ground_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
set(OpenCV_STATIC ON) 
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation kdtree search)# For converting PCL to ROS messages

add_executable(laser_scan_to_bounding_box_node src/laser_scan_to_bounding_box.cpp)

add_executable(lidar_processor_node src/lidar_processor.cpp)

ament_target_dependencies(lidar_processor_node
  rclcpp
  rclpy
  sensor_msgs
  pcl_conversions
  pcl_msgs
  OpenCV
  PCL
  tf2
  tf2_ros
  nav_msgs
  visualization_msgs
  tf2_eigen
  tf2_geometry_msgs
)


ament_target_dependencies(laser_scan_to_bounding_box_node
  rclcpp
  sensor_msgs
  visualization_msgs
  PCL
  pcl_conversions
)

install(TARGETS
  laser_scan_to_bounding_box_node
  lidar_processor_node
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
