cmake_minimum_required(VERSION 3.5)
project(frontier_exploration)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(frontier_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(slam_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_util)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(example_interfaces REQUIRED)

set(dependencies
ament_cmake
rclcpp
rclcpp_action
geometry_msgs
action_msgs
nav2_msgs
rosidl_default_generators
nav2_costmap_2d
frontier_msgs
pcl_ros
sensor_msgs
tf2_ros
tf2
pluginlib
octomap_msgs
slam_msgs
visualization_msgs
eigen3_cmake_module
Eigen3
PCL
octomap
nav2_util
example_interfaces
)


include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
)

#Add libraries
add_library(frontier_exploration_core SHARED plugins/bounded_explore_layer.cpp src/frontier_search.cpp src/frontier_selection.cpp src/planner.cpp)
ament_target_dependencies(frontier_exploration_core SYSTEM ${dependencies})
target_link_libraries(frontier_exploration_core ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})


# Add executable 1
add_executable(explore_client src/explore_client.cpp)
ament_target_dependencies(explore_client rclcpp geometry_msgs std_msgs nav2_msgs nav2_costmap_2d visualization_msgs frontier_msgs rclcpp_action)

# Add executable 2
add_executable(explore_server src/explore_server.cpp)
ament_target_dependencies(explore_server rclcpp nav2_msgs nav2_costmap_2d visualization_msgs frontier_msgs rclcpp_action action_msgs)

# Add executable 5
add_executable(localization_pose_sub src/viz/localization_pose_sub.cpp)
ament_target_dependencies(localization_pose_sub SYSTEM ${dependencies})

# Add executable 5
# add_executable(srv_cl src/srv_cl.cpp)
# ament_target_dependencies(srv_cl SYSTEM ${dependencies})
# srv_cl

install(TARGETS explore_server explore_client frontier_exploration_core
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params include
DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d bounded_explore_layer.xml)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_package()
