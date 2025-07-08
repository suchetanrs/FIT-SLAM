source ~/.bashrc
cd /root/trav_ws/ && colcon build --symlink-install 
source /root/trav_ws/install/setup.bash

cd /home/orb/ORB_SLAM3/ && ./build.sh

cd /root/colcon_ws/ && colcon build --symlink-install
source /root/colcon_ws/install/setup.bash 

cd /root/dev_ws/ && colcon build --symlink-install
source /root/dev_ws/install/setup.bash
