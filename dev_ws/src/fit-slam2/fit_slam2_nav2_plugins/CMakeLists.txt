cmake_minimum_required(VERSION 3.8)
project(fit_slam2_nav2_plugins VERSION "1.1.0")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(fit_slam2_msgs REQUIRED)
# find_package(pcl_ros REQUIRED)

set(dependencies
ament_cmake
rclcpp
std_msgs
geometry_msgs
nav2_msgs
nav_msgs
tf2_ros
nav2_util
sensor_msgs
nav2_costmap_2d
fit_slam2_msgs
# pcl_ros
)

include_directories(
  include
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
)

#Add libraries
add_library(keepout_zones_core SHARED plugins/keepout_layer.cpp)
ament_target_dependencies(keepout_zones_core SYSTEM ${dependencies})

install(TARGETS
ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin)

#  fake_dam_publisher
install(TARGETS keepout_zones_core 
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 # the following line skips the linter which checks for copyrights
 # comment the line when a copyright and license is added to all source files
 set(ament_cmake_copyright_FOUND TRUE)
 # the following line skips cpplint (only works in a git repo)
 # comment the line when this package is in a git repo and when
 # a copyright and license is added to all source files
 set(ament_cmake_cpplint_FOUND TRUE)
 ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_costmap_2d keepout_layer.xml)
ament_package()
