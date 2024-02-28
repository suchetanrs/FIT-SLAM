. /opt/ros/humble/setup.sh
cd /root/slam_ws/
. /opt/ros/humble/setup.sh
colcon build --symlink-install --packages-select rtabmap
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_msgs
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_conversions
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_rviz_plugins
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_sync
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_util
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_odom
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_viz
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_demos
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_python
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_examples
source /root/slam_ws/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_ros
source /root/slam_ws/install/setup.bash
colcon build --symlink-install
