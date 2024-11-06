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
  FRONTIER_POINT_MEDIAN
  # FRONTIER_POINT_INITIAL
)

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable 2
# src/FrontierSearch.cpp
add_executable(explore_server 
src/Frontier.cpp 
src/Nav2Interface.cpp 
src/Helpers.cpp 
src/main.cpp
src/ExplorationBT.cpp 
src/FrontierSearch.cpp 
src/FrontierCostsManager.cpp 
src/CostAssigner.cpp 
src/planners/planner.cpp 
src/planners/FrontierRoadmap.cpp 
src/planners/astar.cpp 
src/FullPathOptimizer.cpp 
src/CostCalculator.cpp 
src/fisher_information/FisherInfoManager.cpp 
src/controllers/recovery_controller.cpp 
src/controllers/initialization_controller.cpp 
src/util/event_logger.cpp 
src/util/rosVisualizer.cpp 
src/nav2_plugins/lethal_marker.cpp
)
ament_target_dependencies(explore_server ${dependencies})
target_link_libraries(explore_server taskAllocator_lib ${PCL_LIBRARIES})

#Add libraries
add_library(lethal_marker_core SHARED src/nav2_plugins/lethal_marker.cpp)
ament_target_dependencies(lethal_marker_core SYSTEM ${dependencies})

add_library(frontier_roadmap_planner_core SHARED
  src/nav2_plugins/roadmap_planner_plugin.cpp
)
ament_target_dependencies(frontier_roadmap_planner_core ${dependencies})
target_compile_definitions(frontier_roadmap_planner_core PUBLIC "PLUGINLIB_DISABLE_BOOST_FUNCTIONS")

# -------------------------- Test files ----------------------------
# add_executable(main_fisher_information_manager src/FisherInfoManager.cpp src/rosVisualizer.cpp src/Frontier.cpp tests/main_fisher_information_manager.cpp)
# ament_target_dependencies(main_fisher_information_manager ${dependencies})
# target_link_libraries(main_fisher_information_manager taskAllocator_lib ${PCL_LIBRARIES})

add_executable(nav2_interface_main src/Nav2Interface.cpp tests/nav2_interface_main.cpp)
ament_target_dependencies(nav2_interface_main ${dependencies})
# target_link_libraries(nav2_interface_main taskAllocator_lib ${PCL_LIBRARIES})

add_executable(logger_test tests/logger_test.cpp src/Frontier.cpp)
ament_target_dependencies(logger_test ${dependencies})
# target_link_libraries(logger_test taskAllocator_lib ${PCL_LIBRARIES})

add_executable(frontier_class tests/frontier_class.cpp src/Frontier.cpp)
ament_target_dependencies(frontier_class ${dependencies})
# target_link_libraries(frontier_class taskAllocator_lib ${PCL_LIBRARIES})

add_executable(main_fim_computation tests/main_fim_computation.cpp src/Helpers.cpp src/Frontier.cpp src/util/event_logger.cpp)
ament_target_dependencies(main_fim_computation ${dependencies})
# target_link_libraries(main_fim_computation taskAllocator_lib ${PCL_LIBRARIES})

add_executable(fim_viz tests/fim_viz.cpp src/Helpers.cpp src/Frontier.cpp src/util/event_logger.cpp)
ament_target_dependencies(fim_viz ${dependencies})
# target_link_libraries(fim_viz taskAllocator_lib ${PCL_LIBRARIES})

add_executable(load_fi_lookup src/fisher_information/FisherInfoManager.cpp src/fisher_information/LoadLookupMain.cpp 
                                  src/Frontier.cpp src/Helpers.cpp src/planners/FrontierRoadmap.cpp src/util/event_logger.cpp src/planners/astar.cpp src/util/rosVisualizer.cpp)
ament_target_dependencies(load_fi_lookup ${dependencies})
# target_link_libraries(fim_viz taskAllocator_lib ${PCL_LIBRARIES})

add_executable(gen_fi_lookup src/fisher_information/FisherInfoManager.cpp src/fisher_information/GenerateLookupMain.cpp 
                                  src/Frontier.cpp src/Helpers.cpp src/planners/FrontierRoadmap.cpp src/util/event_logger.cpp src/planners/astar.cpp src/util/rosVisualizer.cpp)
ament_target_dependencies(gen_fi_lookup ${dependencies})

add_executable(roadmap_vertices_saver tests/frontier_roadmap_tests/roadmap_vertices_saver.cpp)
ament_target_dependencies(roadmap_vertices_saver ${dependencies})

# -------------------------- Comparision files ---------------------
add_executable(explored_map_counter comparision_scripts/explored_map_counter.cpp)
ament_target_dependencies(explored_map_counter ${dependencies})

add_executable(exploration_path_followed comparision_scripts/exploration_path_followed.cpp)
ament_target_dependencies(exploration_path_followed ${dependencies})

add_executable(exploration_path_followed_tf comparision_scripts/exploration_path_followed_tf.cpp)
ament_target_dependencies(exploration_path_followed_tf ${dependencies})

# main_fisher_information_manager
install(TARGETS nav2_interface_main lethal_marker_core frontier_roadmap_planner_core explored_map_counter exploration_path_followed exploration_path_followed_tf
DESTINATION lib/${PROJECT_NAME}
)
# ------------------------------------------------------------------


install(TARGETS explore_server fim_viz load_fi_lookup gen_fi_lookup roadmap_vertices_saver
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params include
DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
pluginlib_export_plugin_description_file(nav2_costmap_2d xml/lethal_marker.xml)
pluginlib_export_plugin_description_file(nav2_core xml/roadmap_planner.xml)
ament_package()
