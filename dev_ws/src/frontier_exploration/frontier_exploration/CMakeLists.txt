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
find_package(slam_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_util)
find_package(PCL REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(frontier_multirobot_allocator REQUIRED)
find_package(behaviortree_cpp REQUIRED)

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
frontier_msgs
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
frontier_multirobot_allocator
behaviortree_cpp
)

# Add compile definitions for visualization flags
add_compile_definitions(
  # FRONTIER_POINT_CENTROID
  FRONTIER_POINT_INITIAL
)

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable 2
# src/FrontierSearch.cpp
add_executable(explore_server src/ExplorationBT.cpp src/Nav2Interface.cpp plugins/bounded_explore_layer.cpp src/FrontierSearch.cpp src/FrontierCostsManager.cpp 
               src/planners/planner.cpp src/planners/rrt.cpp src/rosVisualizer.cpp src/CostCalculator.cpp src/main.cpp
               src/Frontier.cpp src/Helpers.cpp src/planners/FrontierRoadmap.cpp src/planners/astar.cpp src/FisherInfoManager.cpp)
ament_target_dependencies(explore_server ${dependencies})
target_link_libraries(explore_server taskAllocator_lib ${PCL_LIBRARIES})

# -------------------------- Test files ----------------------------
# add_executable(main_fisher_information_manager src/FisherInfoManager.cpp src/rosVisualizer.cpp src/Frontier.cpp tests/main_fisher_information_manager.cpp)
# ament_target_dependencies(main_fisher_information_manager ${dependencies})
# target_link_libraries(main_fisher_information_manager taskAllocator_lib ${PCL_LIBRARIES})

add_executable(nav2_interface_main src/Nav2Interface.cpp tests/nav2_interface_main.cpp)
ament_target_dependencies(nav2_interface_main ${dependencies})
# target_link_libraries(nav2_interface_main taskAllocator_lib ${PCL_LIBRARIES})

# main_fisher_information_manager
install(TARGETS nav2_interface_main
DESTINATION lib/${PROJECT_NAME}
)
# ------------------------------------------------------------------


install(TARGETS explore_server
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params include
DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_package()
