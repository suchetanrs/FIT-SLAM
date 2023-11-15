cmake_minimum_required(VERSION 3.5)
project(traversability_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rtabmap_msgs REQUIRED)

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/PCL2WithNodeID.msg"
DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs rtabmap_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
