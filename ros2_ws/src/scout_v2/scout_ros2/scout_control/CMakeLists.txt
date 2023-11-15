cmake_minimum_required(VERSION 3.5)
project(scout_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(octomap REQUIRED)

include_directories(
  include
)

set(DEPENDENCIES
  nav_msgs
  std_msgs
  ament_cmake
  rclcpp
  rclcpp_action
  geometry_msgs
  action_msgs
  nav2_msgs
  rosidl_default_generators
  nav2_costmap_2d
  sensor_msgs
  tf2_ros
  tf2
  pluginlib
  octomap_msgs
  visualization_msgs
  eigen3_cmake_module
  Eigen3
  octomap
  nav2_util
)

add_executable(scout_path_follower src/scout_path_follower.cpp)
ament_target_dependencies(scout_path_follower SYSTEM ${DEPENDENCIES})

add_executable(scout_path_corrector src/scout_path_corrector.cpp)
ament_target_dependencies(scout_path_corrector SYSTEM ${DEPENDENCIES})

install(TARGETS scout_path_follower scout_path_corrector
  DESTINATION lib/${PROJECT_NAME})

ament_package()
