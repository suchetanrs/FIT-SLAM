source ~/.bashrc
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
cd /root/dev_ws/ && colcon build --symlink-install
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install