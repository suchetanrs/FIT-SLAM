. /opt/ros/humble/setup.sh
cd /root/slam_ws/
. /opt/ros/humble/setup.sh
colcon build --symlink-install --packages-select rtabmap
colcon build --symlink-install --packages-select rtabmap_msgs
colcon build --symlink-install --packages-select rtabmap_conversions
colcon build --symlink-install --packages-select rtabmap_rviz_plugins
colcon build --symlink-install --packages-select rtabmap_sync
colcon build --symlink-install --packages-select rtabmap_util
colcon build --symlink-install --packages-select rtabmap_odom
colcon build --symlink-install --packages-select rtabmap_viz
colcon build --symlink-install --packages-select rtabmap_demos
colcon build --symlink-install --packages-select rtabmap_python
colcon build --symlink-install --packages-select rtabmap_examples
colcon build --symlink-install --packages-select rtabmap_ros
colcon build --symlink-install
