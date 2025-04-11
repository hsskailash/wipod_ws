cmake_minimum_required(VERSION 3.5)
project(wipod_nav)

# Find the required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_voxel_grid REQUIRED) 
find_package(nav2_smac_planner REQUIRED)
find_package(teb_local_planner REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(nav2_velocity_smoother REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Install directories
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY
  src/
  DESTINATION lib/${PROJECT_NAME}
)

# Finalize package
ament_package()
