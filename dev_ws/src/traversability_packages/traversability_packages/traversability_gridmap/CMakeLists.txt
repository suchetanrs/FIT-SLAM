cmake_minimum_required(VERSION 3.8)
project(traversability_gridmap)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pluginlib REQUIRED)
find_package(traversability_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

find_package(PCL REQUIRED)
grid_map_package()

set(dependencies
  grid_map_msgs
  grid_map_ros
  rclcpp
  sensor_msgs
  std_msgs
  Eigen3
  visualization_msgs
  pcl_ros
  pcl_conversions
  PCL
  rclcpp_lifecycle
  nav2_costmap_2d
  grid_map_ros
  grid_map_cv
  cv_bridge
  pluginlib
  traversability_msgs
  nav_msgs
)


include_directories(
  include
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

#Executable 1
add_executable(pointcloud_transformer src/pointcloud_transformer.cpp src/global_traversability_map.cpp)
ament_target_dependencies(pointcloud_transformer SYSTEM ${dependencies})
target_link_libraries(pointcloud_transformer ${PCL_LIBRARIES})

#Executable 2
add_executable(traversability_server src/traversability_server.cpp)
ament_target_dependencies(traversability_server SYSTEM ${dependencies})

#Executable 3
add_executable(traversability_thresholded src/traversability_thresholded.cpp)
ament_target_dependencies(traversability_thresholded SYSTEM ${dependencies})


#Library 1
add_library(traversability_gridmap_core SHARED plugins/traversability_layer.cpp src/traversabilityGridMod.cpp src/global_traversability_map.cpp)
ament_target_dependencies(traversability_gridmap_core ${dependencies})
target_link_libraries(traversability_gridmap_core ${PCL_LIBRARIES})


install(TARGETS pointcloud_transformer traversability_gridmap_core traversability_thresholded traversability_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d traversability_layer.xml)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})

ament_package()
