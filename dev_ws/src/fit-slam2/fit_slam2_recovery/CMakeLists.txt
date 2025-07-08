cmake_minimum_required(VERSION 3.5)
project(fit_slam2_recovery)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(fit_slam2_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(slam_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_util)
find_package(PCL REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(roadmap_explorer REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(dependencies
ament_cmake
rclcpp
rclcpp_action
geometry_msgs
action_msgs
nav2_msgs
rosidl_default_generators
nav2_costmap_2d
fit_slam2_msgs
pcl_ros
sensor_msgs
tf2_ros
tf2
pluginlib
slam_msgs
visualization_msgs
eigen3_cmake_module
Eigen3
PCL
nav2_util
example_interfaces
behaviortree_cpp
yaml-cpp
roadmap_explorer
)

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO environment variable not set—are you sourcing your ROS2 installation?")
endif()

string(TOUPPER "$ENV{ROS_DISTRO}" ROS_DISTRO_UPPER)
message(STATUS "Building for ROS distro: ${ROS_DISTRO_UPPER}")
add_compile_definitions(ROS_DISTRO_${ROS_DISTRO_UPPER})

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(fit_slam2_recovery SHARED 
src/recovery_controller.cpp
src/bt_plugin.cpp)
ament_target_dependencies(fit_slam2_recovery ${dependencies})

install(TARGETS fit_slam2_recovery
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_export_libraries(fit_slam2_recovery)
pluginlib_export_plugin_description_file(roadmap_explorer fit_slam2_recovery.xml)
ament_package()
