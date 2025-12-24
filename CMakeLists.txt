cmake_minimum_required(VERSION 3.8)
project(laser_uav_px4_api)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(laser_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/api_node.cpp
)

# Specify dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  laser_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  px4_msgs
  Eigen3
)

# Register the component
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "laser_uav_px4_api::LaserUavPx4Api"
  EXECUTABLE laser_uav_px4_api_node
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install params files
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()