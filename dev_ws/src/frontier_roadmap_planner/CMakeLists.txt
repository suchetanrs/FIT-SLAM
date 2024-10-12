cmake_minimum_required(VERSION 3.5)
project(frontier_roadmap_planner)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package() #Calls the nav2_package.cmake file
add_compile_options(-O3)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies ament_cmake
  builtin_interfaces
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2_ros
)


add_library(${library_name} SHARED
  src/roadmap_planner_plugin.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB_DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core roadmap_planner.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES roadmap_planner.xml
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()