cd launch/
ls
cat scout_base_description.launch.py 
cd ../..
ls
cd ..
sl
ls
cd scout_gazebo/
ls
cd launch/
ls
nano spawn_scout.launch.py 
cat spawn_scout.launch.py 
ros2 launch scout_gazebo start_world.launch.py 
ros2 launch scout_gazebo scout_simu.launch.py 
export SCOUT_X=1.0
export SCOUT_Y=1.0
ros2 launch scout_gazebo scout_simu.launch.py 
tmux
nautilus .
ros2 run tf2_tools view_frames 
nautilus .
rviz2
cd ros2_ws/
vsc
ros2 launch scout_gazebo spawn_scout.launch.py 
ros2 launch scout_gazebo scout_simu.launch.py 
export SCOUT_X=1.0
export SCOUT_Y=1.0
ros2 launch scout_gazebo scout_simu.launch.py 
ros2 launch scout_gazebo start_world.launch.py 
tmux
ls
tmux
cd dev_ws/
vsc
firefox
snap install firefox
firefox
chrome
sudo apt-get install libxss1 libappindicator1 libindicator7
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo apt install ./google-chrome*.deb
ls
cd 
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb https://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'
sudo apt-get update
sudo apt-get install google-chrome-stable
google-chrome-stable 
google-chrome-stable --no-sandbox
cd dev_ws/
vsc
google-chrome-stable --no-sandbox
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb https://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'
sudo apt-get update
sudo apt-get install google-chrome-stable
ls
cd
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb https://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'
sudo apt-get update
sudo apt-get install google-chrome-stable
ls
rm -rf core.*
ls
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb https://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'
sudo apt-get update
sudo apt-get install google-chrome-stable
cd dev_ws/
vsc
google-chrome-stable --no-sandbox
cd /root/dev_ws/src/frontier_exploration/frontier_exploration/src/planners/
g++ -o astar_on_graph2.cpp astar2
g++ -o astar2 astar_on_graph2.cpp
./astar2 
g++ -o astar2 astar_on_graph2.cpp
./astar2 
cd /root/dev_ws/src/frontier_exploration/frontier_exploration/src/planners/
ls
clear
g++ -o astar astar_on_graph.cpp 
./astar 
g++ -o astar astar_on_graph.cpp 
./astar 
clear
g++ -o astar astar_on_graph.cpp 
clear
./astar 
ls
cd dev_ws/
vsc
colcon build --symlink-install
source install/setup.bash
ros2 run exploration_sim explore_sim 
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
kill -9 %1
ros2 run exploration_sim explore_sim 
kill -9 %1
ros2 run exploration_sim explore_sim 
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
cd -
ros2 topic echo /scout_2/connecting_cells
rviz2
bash
cd dev_ws/
colcon build --symlink-install
./shell_scripts/3_exploration.sh 
nano shell_scripts/3_exploration.sh 
./shell_scripts/3_exploration.sh 
nano shell_scripts/3_exploration.sh 
./shell_scripts/3_exploration.sh 
bash
cd dev_ws/
vscs
vsc
cd
tmux
rviz2
ros2 run exploration_sim explore_sim 
cd dev_ws/
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
ls
cd dev_ws/
colcon build --symlink-install
bash
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make
make install
cd dev_ws/
colcon build --symlink-install
cd
./shell_scripts/3_exploration.sh 
cd dev_ws/
colcon build --symlink-install
./../shell_scripts/3_exploration.sh 
colcon build --symlink-install
./../shell_scripts/3_exploration.sh 
colcon build --symlink-install
./../shell_scripts/3_exploration.sh 
./shell_scripts/3_exploration.sh 
bash
cd dev_ws/
vsc
ls
rm -rf go2_sim.tar.xz unitree_guide2.tar.xz 
ls
cd src/
ls
rm -rf go1_sim/ unitree_guide2/
ls
cd ..
ls
clear
bash
clear
ls
clear
ls
clear
ls
clear
ls
cd dev_ws/
ls
cd src/
ls
cd frontier_exploration/
ls
rm -rf COLCON_IGNORE 
ls
cd ..
ls
cd exploration_sim/
rm -rf COLCON_IGNORE 
ls
cd ..
ls
clear
ls
cd ..
ls
tmux
ros2 launch go2_gazebo spawn_go2.launch.py 
nano dev_ws/src/go2_sim/go2_gazebo/launch/spawn_go2.launch.py 
ros2 launch go2_gazebo spawn_go2.launch.py 
nano dev_ws/src/go2_sim/go2_gazebo/launch/spawn_go2.launch.py 
ros2 launch go2_gazebo spawn_go2.launch.py 
nano dev_ws/src/go2_sim/go2_gazebo/launch/spawn_go2.launch.py 
ros2 launch go2_gazebo spawn_go2.launch.py 
nano dev_ws/src/go2_sim/go2_gazebo/launch/spawn_go2.launch.py 
ros2 launch go2_gazebo spawn_go2.launch.py 
ros2 launch go2_gazebo spawn_go.launch
source dev_ws/install/setup.bash
ros2 launch go2_gazebo spawn_go.launch
ros2 launch go2_gazebo start_world.launch.py
ls
ros2 run unitree_guide2 junior_ctrl 
cd dev_ws/
ls
colcon build --symlink-install --packages-select ros2_unitree_legged_
colcon build --symlink-install --packages-select ros2_unitree_legged_msgs 
ros2 run unitree_guide2 junior_ctrl 
ls
cd dev_ws/
vsc
cd
cd /root/dev_ws/src/go2_sim/go2_description/xacro
vsc
cd
cd dev_ws/
vsc
ls
cd src/
ls
cd go1_sim/
ls
cd ..
ls
rm -rf go2_sim/
ls
cd ..
ls
rm -rf build/ install/ log/
colcon build --symlink-install
ros2 topic list
ros2 topic echo /joint_states 
ros2 topic list
clear
ls
cd dev_ws/
ls
cd src/
ls
cd ..
ls
unzip go2_sim.tar.xz 
apt-get install unzip
unzip go2_sim.tar.xz 
tar -xvzf go2_sim.tar.xz src/
ls
cd src/
ls
cd ..
ls
tar -xvzf go2_sim.tar.xz
sudo apt install xz-utils
tar -Jxvf unitree_guide2.tar.xz 
ls
mv unitree_guide2 src/
ls
tar -Jxvf go2_sim.tar.xz 
ls
mv go2_sim src/
ls
colcon build --symlink-install --packages-select unitree_guide2 
colcon build --symlink-install --packages-select ros2_unitree_legged_msgs 
colcon build --symlink-install
source install/setup.bash
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_exploration/src/planners/
cd ..
ls
cd ..
ls
cd ..
ls
cd frontier_multirobot_allocator/
ls
cmake ..
make
cd build/
make install
cd
ls
cd dev_ws/
colcon build --symlink-install
ls
cd src/
ls
cd exploration_sim/
ls
touch COLCON_IGNORE
cd ..
ls
cd frontier_exploration/
touch COLCON_IGNORE
cd
cd dev_ws/
colcon build --symlink-install
ls
rm -rf build/ install/ log/
colcon build --symlink-install
rm -rf build/ install/ log/
colcon build --symlink-install --packages-select ros2_unitree_legged_msgs 
colcon build --symlink-install
apt-get install lcm
pip3 install lcm
colcon build --symlink-install
ls
cd src/
ls
cd ..
ls
wget https://github.com/lcm-proj/lcm/archive/refs/tags/v1.5.0.zip
ls
unzip v1.5.0.zip 
ls
cd lcm-1.5.0/
ls
mkdir build
cd build/
cmake ..
make
make install
ls
cd
cd dev_ws/
ls
colcon build --symlink-install
ls
clear
ls
clear
ls
clear
ls
clear
ls
clear
ls
clear
ls
clear
ls
clear
ls
clear
ls
clear
ls
clear
tmux
ros2 launch go2_gazebo start_world.launch.py 
kill -9 %1
ros2 launch go2_gazebo spawn_go2.launch
cd dev_ws/
source install/setup.bash
ros2 launch go2_gazebo spawn_go2.launch.py 
ls
cd src/
ls
cd shared_pkgs/
vsc
ros2 launch go2_gazebo spawn_go2.launch.py 
source dev_ws/install/setup.bash
ros2 run unitree_guide2 junior_ctrl 
cd dev_ws/
ls
colcon build --symlink-install
ros2 run unitree_guide2 junior_ctrl 
ros2 topic list
ros2 topic echo /RR_calf_controller/state
ros2 topic hz /RR_calf_controller/state
ros2 run unitree_guide2 junior_ctrl 
htop
top
htop
jtop
top
ros2 topic hz /RR_calf_controller/state
ros2 topic hz /RR_calf_controller/command 
top
clear
apt-get install ros-humble-plot-juggler
apt-get install ros-humble-plotjuggler
apt-get install ros-humble-plotjuggler-ros
ros2 run plotjuggler plotjuggler 
ls
cd dev_ws/
ls
colcon build --symlink-install
rm -rf build/ install/ log/
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make
make install
cd
cd dev_ws/
colcon build --symlink-install
ls
cd 
ls
unzip lcm-1.5.0
ls
rm -rf lcm-1.5.0.zip 
ls
cd lcm-1.5.0/
ls
cd build/
cmake ..
make
make install
cd
cd dev_ws/
colcon build --symlink-install
tmux
./shell_scripts/3_exploration.sh 
ls
cd dev_ws/
ros2 run exploration_sim explore_sim 
rviz2
cd dev_ws/
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make
make install
cd d
cd dev_ws/
vsc
tmux
ls
cd dev_ws/
vsc
clear
ls
clear
ls
clear
ls
clear
ls
clear
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
kill -9 %1
ros2 run exploration_sim explore_sim 
ls
clear
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
./shell_scripts/3_exploration.sh 
rviz2
cd dev_ws/
colcon build --symlink-install
ls
tmux
cd .ros/
ls
cd log/
ls
rm -rf *
ls
cd ..
ls
cd log/
ls
rm -rf *
ls
clear
ls
clear
ls
cd ..
ls
zip log/ log.zip
ls
apt-get install zip
zup log/ -o log.zip
zip log/ -o log.zip
zip log/
ls
cd log/
ls
cd ..
ls
zip log
man zip
zip -r logs_2_pressed.zip log
ls
ros2 launch go2_gazebo spawn_go2.launch.py 
clear
ros2 launch go2_gazebo spawn_go2.launch.py 
clear
ros2 launch go2_gazebo spawn_go2.launch.py 
ros2 run unitree_guide2 junior_ctrl 
clear
ros2 run unitree_guide2 junior_ctrl 
ls
cd dev_ws/
ls
cd src/
ls
rm -rf shared_pkgs/
ls
cd
ls
rm -rf shared_pkgs-20240702T154954Z-001.zip
ls
unzip shared_pkgs-20240703T072847Z-001.zip 
apt-get install unzip
unzip shared_pkgs-20240703T072847Z-001.zip 
ls
cd sh
mv shared_pkgs dev_ws/src/
ls
cd lcm-1.5.0/
cd build/
cmake ..
make 
make install
cd
cd dev_ws/
ls
cd src/
ls
cd ..
ls
colcon build --symlink-install
ls
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
cd dev_ws/
colcon build --symlink-install
ls
cd src/
ls
cd shared_pkgs/
la
unzip *
ls
unzip ros2_unitree_legged_*
ls
unzip go1_sim.zip 
ls
unzip quadruped_sim.zip 
ls
unzip ros2_unitree_legged_controller.zip 
ls
unzip ros2_unitree_legged_msgs.zip 
ls
unzip unitree_guide2.zip 
ls
rm -rf go1_sim.zip quadruped_sim.zip ros2_unitree_legged_controller.zip ros2_unitree_legged_msgs.zip unitree_guide2.zip 
ls
cd ../..
colcon build --symlink-install
ls
cd src/
ls
cd shared_pkgs/
ls
rm -rf quadruped_sim/
ls
cd ../..
colcon build --symlink-install
ls
tmux
ros2 launch go2_gazebo spawn_go2.launch.py 
ros2 run unitree_guide2 junior_ctrl 
ros2 bag record /joint_states /RL_calf_joint/*
ros2 bag record /joint_states /RL_calf_joint/command
ls
rm -rf rosbag2_2024_07_03-13_34_52/
ls
cd .ros/
ls
ros2 topic list
ros2 bag record /RL_calf_controller/command /RL_calf_controller/state /RL_calf_controller/transition_event /joint_states
cd .ros/
ls
cd log/
ls
rm -rf *
ls
cd ..
ls
zip -r -v log_2_dabake_1.zip log
apt-get install zip
zip -r -v log_2_dabake_1.zip log
ls
cd log
ls
rm -rf *
ls
cd ..
ls
zip -r -v log_2_dabake_2.zip log
ls
cd log/
ls
rm -rf *
ls
cd ..
ls
zip -r -v log_2_dabake_5_dabake_bag_ke_saath.zip log
ls
zip -r -v rosbag2_2_dabake_5_dabake rosbag2_2024_07_03-13_36_13/
ls
ls
tmux
ls
ros2 launch go2_gazebo spawn_go2.launch.py 
ros2 run unitree_guide2 junior_ctrl 
ros2 bag record --all 
ls
rm -rf rosbag2_2024_07_03-23_58_26/
ros2 bag record --all 
ls
cd lcm-1.5.0/
s
cd build/
cmake ..
make install
cd
sl
ls
cd dev_ws/
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
cd dev_ws/
colcon build --symlink-install
source install/setup.bash
cd
ls
tmux
ros2 launch scout_gazebo start_world.launch.py 
ros2 launch scout_gazebo scout_simu.launch.py 
export SCOUT_X= 1.0
export SCOUT_X=1.0
export SCOUT_Y=1.0
ros2 launch scout_gazebo scout_simu.launch.py 
echo $ROS_DOMAIN_ID 
rviz2
ls
tmux
ros2 launch scout_gazebo start_world.launch.py 
ros2 launch gazebo_ros gazebo.launch.py 
kill -9 %1
bash
ros2 run exploration_sim explore_sim 
source dev_ws/install/setup.bash
ros2 launch go1_gazebo spawn_go1.launch.py 
ls
clear
cd dev_ws/
ls
cd src/
ls
cd go1/
ls
nano go1_sim/go1_gazebo/launch/spawn_go1.launch.py 
cd
ros2 launch go1_gazebo spawn_go1.launch.py 
rviz2
./shell_scripts/3_exploration.sh 
ls
cd lcm-1.5.0/
cd build/
ls
cmake ..
make install
cd
cd dev_ws/
ls
cd
apt-get install unzip
unzip working.zip 
ls
cd dev_ws/
ls
cd ..
mv working.zip dev_ws/
ls
cd dev_ws/
ls
mv working.zip src/
ls
cd src/
ls
rm -rf shared_pkgs/
ls
mkdir go1
cd go1/
ls
cd ..
mv working.zip go1/
ls
cd go1/
ls
unzip working.zip 
ls
cd ..
ls
cd ..
ls
colcon build --symlink-install
ls
rm -rf build/ install/ log/
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
cd dev_ws/
colcon build --symlink-install
clear
colcon build --symlink-install
ls
cd dev_ws/
vsc
tmux
ls
clear
ros2 run unitree_guide2 junior_ctrl 
cd dev_ws/
colcon build --symlink-install
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 run unitree_guide2 junior_ctrl 
ls
c
cd
ls
code .
vsc
ros2 run unitree_guide2 junior_ctrl 
ls
cd ros2_
cd ros2_ws/
ls
cd src/
ls
cd
cd dev_ws/
ls
cd src/
ls
cd go1/
ls
rm -rf working.zip 
ls
cd ..
ls
nautilus .
ls
rm -rf go1.zip 
nautilus .
ls
clear
ls
clear
ros2 launch scout_navigation2 navigation.launch.py 
ls
cd ros2_ws/src/scout_v2/scout_navigation2/
ls
cd launch/
ls
nano navigation.launch.py 
echo $SCOUT_NAMESPACE 
export SCOUT_NAMESPACE=""
ros2 launch scout_navigation2 navigation.launch.py 
nano navigation.launch.py 
ros2 launch scout_navigation2 navigation.launch.py 
cd ../..
ls
zip scout_navigation2/
apt-get install scout_navigation2/
apt-get install zip
zip scout_navigation2/
nautilus .
ros2 topic echo /scan
cd ros2_ws/
vsc
ros2 launch slam_toolbox online_async_launch.py params_file:=/root/dev_ws/src/go1/go1_sim/go1_description/config/mapper
_params.yaml use_sim_time:=true 
ros2 launch slam_toolbox online_async_launch.py params_file:=/root/dev_ws/src/go1/go1_sim/go1_description/config/mapper_params.yaml use_sim_time:=true 
cd dev_ws/
colcon build --symlink-install
ros2 launch slam_toolbox online_async_launch.py params_file:=/root/dev_ws/src/go1/go1_sim/go1_description/config/mapper_params.yaml use_sim_time:=true 
ros2 run teleop_twist_keyboard 
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 launch slam_toolbox online_async_launch.py params_file:=/root/dev_ws/src/go1/go1_sim/go1_description/config/mapper_params.yaml use_sim_time:=true 
cd
nano /opt/ros/humble/share/slam_toolbox/launch/online_async_launch.py 
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/root/dev_ws/src/go1/go1_sim/go1_description/config/mapper_params.yaml use_sim_time:=true 
ros2 launch go1_gazebo spawn_go1.launch.py 
ls
nano ros2_ws/src/scout_v2/
cd ros2_ws/src/scout_v2/
ls
nano scout_gazebo/launch/start_world.launch.py 
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
cd
cd dev_ws/
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
apt-get install ros-humble-slam-toolbox
clear
cd
ls
clear
cd lcm-1.5.0/
cd build/
cmake ..
make install
ros2 run unitree_guide2 junior_ctrl 
ros2 launch slam_toolbox online_async_launch.py params_file:=/root/dev_ws/src/go1/go1_sim/go1_description/config/mapper_params.yaml 
ros2 launch slam_toolbox online_async_launch.py params_file:=/root/dev_ws/src/go1/go1_sim/go1_description/config/mapper_params.yaml use_sim_time:=true
bash
ros2 topic list | grep cmd_vel
ros2 topic list | grep cmd_vel_nav
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 launch go1_gazebo spawn_go1.launch.py 
ros2 launch scout_gazebo start_world.launch.py 
cd dev_ws/
vsc
ros2 launch scout_gazebo start_world.launch.py 
colcon build --symlink-install
ros2 launch scout_gazebo start_world.launch.py 
kill -9 %1
ros2 launch scout_gazebo start_world.launch.py 
ros2 run tf2_ros tf2_echo odom laser_link
ros2 topic echo /odom
ros2 topic echz /odom
ros2 topic hz /odom
ls
ros2 run tf2_tools view_frames 
nautilus .
ros2 topic hz /odom
ros2 topic echo /odom
ros2 run tf2_tools view_frames 
nautilus .
nano go1_odom.py
python3 go1_
python3 go1_odom.py 
clear
tmux
ls
cd dev_ws/
ls
cd src/
ls
cd unitree/
ls
cd ..
ls
colcon build --symlink-install
cd
cd lcm-1.5.0/
cd build/
cmake ..
make install
cd
cd dev_ws/
colcon build --symlink-install
ros2 run unitree_guide2 junior_ctrl 
cd dev_ws/
ls
ros2 launch go1_gazebo spawn_go1.launch.py 
ros2 launch go1_navigation navigation.launch.py 
ls
bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
cd dev_ws/
ls
cd src/
ls
cd unitree/
ls
cd README.md 
ls
nano README.md 
ls
nano README.md 
cd go1_sim/
ls
cd go1_navigation/
ls
cd params/
ls
nano nav2_params.yaml 
vsc
ls
cd dev_ws/
ls
apt-get install unzip
unzip unitree_gazebo_ros2.zip 
ls
rm -rf go1_sim/ ros2_unitree_legged_*
ls
rm -rf unitree_guide2/
ls
mv unitree_gazebo_ros2.zip src/
ls
cd src/
ls
mkdir unitree
ls
mv unitree_gazebo_ros2.zip unitree
ls
cd unitree/
ls
unzip unitree_gazebo_ros2.zip 
ls
cd ..
ls
cd ..
ls
colcon build --symlink-install
ls
rm -rf build/ install/ log/
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
cd dev_ws/
colcon build --symlink-install
tmux
ls
cd dev_ws/
vsc
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
./shell_scripts/3_exploration.sh 
cd dev_ws/
colcon build --symlink-install
cd
./shell_scripts/3_exploration.sh 
rviz2
ls
cd dev_ws/
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd dev_ws/
colcon build --symlink-install
cd
ls
cd hotspot/
ls
./hotspot-v1.4.1-292-g5a974a4-x86_64.AppImage 
cd
cd sh
cd shell_scripts/
./install_hotspot.sh 
nano install_hotspot.sh 
./install_hotspot.sh 
cd
cd hotspot/
./hotspot-v1.4.1-292-g5a974a4-x86_64.AppImage 
ls
cd 
cd sh
cd shell_scripts/
ls
nano install_hotspot.sh 
cd
cd hotspot/
./hotspot-v1.4.1-292-g5a974a4-x86_64.AppImage 
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
cd hotspot/
ls
./hotspot-v1.4.1-292-g5a974a4-x86_64.AppImage 
pidof explore_server
rviz2
./shell_scripts/3_exploration.sh 
ls
./shell_scripts/3_exploration.sh 
htop
top
pidof explore_server
cd dev_ws/
colcon build --symlink-install
pidof explore_server
colcon build --symlink-install
top
colcon build --symlink-install
clear
ros2 run exploration_sim explore_sim 
kill -9 %1
cd
cd shell_scripts/
ls
nano install_hotspot.sh 
./install_hotspot.sh 
cd
ls
cd hotspot/
ls
./hotspot-v1.4.1-292-g5a974a4-x86_64.AppImage 
ls
rm -rf perf.data perf.data.old 
ls
cd
ros2 run exploration_sim explore_sim 
cd dev_ws/
vsc
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
kill -9 %1
ros2 run exploration_sim explore_sim 
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
colcon build --symlink-install
ros2 run exploration_sim explore_sim 
clear
ls
clear
cd dev_ws/
vsc
tmux
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd dev_ws/
vsc
clear
vsc
colcon build --symlink-install
clear
tmux
clear
ls
clear
cd dev_ws/
vsc
clear
ls
cd dev_ws/src/
ls
vsc
ls
cd ros2_
cd ros2_ws/
ls
clear
ls
clear
ls
clear
ls
cd
ls
cd dev_ws/
lsd
ls
cd
ls
rm -rf ros2_ws/ slam_ws/
ls
rm -rf building_editor_models/
ls
gazebo
rm -rf turtlebot3/ v1.5.0.zip shared_pkgs-20240703T072847Z-001.zip foxglove-studio-2.7.0-linux-amd64.deb 
ls
rm -rf openrmf_ws/
ls
clear
ls
rm -rf go1_
rm -rf go1_odom.py 
s
ls
rm -rf refactored_old/
ls
rm -rf ros2_unitree_legged_*
ls
rm -rf scout_1*
ls
rm -rf scout_*
ls
rm -rf frames_2024-0*
ls
rm -rf perf.data.old 
ls
rm -rf right_*
ls
rm -rf robot_0_12904_ours_frontier_map_data_coverage.csv 
ls
rm -rf map 
ls
rm -rf data_stereo.py compute.py coloured.cpp colo
ls
rm -rf 12904_ours_frontier_map_data_coverage.csv 
ls
rm -rf gazebo_models_worlds_collection/
ls
clear
ls
cd dev_ws/
ls
rm -rf build/ install/ log/
rm -rf core.*
ls
rm -rf cpu.py 
l;s
ls
rm -rf call.py full_config.rviz 
ls
rm -rf scout_2_12904_ours_frontier_map_data_coverage.csv 
ls
colcon build --symlink-install
rosdep install --from-paths src --ignore-src -r -y --simulate
vsc
ls
cd src/
ls
rm -rf traversability_packages/
ls
rm -rf unitree/
ls
cd ..
ls
colcon build --symlink-install
rosdep install --from-paths src --ignore-src -r -y --simulate
ros2 topic list
nano ~/.bashrc 
cd dev_ws/
colcon build --symlink-install
rm -rf build/ install/ log/
colcon build --symlink-install
rosdep install --from-paths src --ignore-src -r -y --simulate
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
cd dev_ws/
colcon build --symlink-install
source install/setup.bash
ros2 launch frontier_exploration exploration
ros2 launch frontier_exploration exploration.launch.py 
ros2 topic list
ls
cd dev_ws/
colcon build --symlink-install
vsc
nano ~/.bashrc 
bash
ls
bash
rviz2
apt-get install ros-humble-navigation2
rviz2
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd dev_ws/
ls
colcon build --symlink-install
ros2 launch robot_navigation2 navigation.launch.py
apt-get install ros-humble-navigation2
ros2 launch robot_navigation2 navigation.launch.py
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation.launch.py
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 topic list
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0\
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
colcon build --symlink-install
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
colcon build --symlink-install
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ls
cd dev_ws/
ls
clear
ls
vsc
ls
cd src/
ls
cd ..
ls
cd src/
lks
ls
cd ..
ls
colcon build --symlink-install
ls
cd src/
ls
cd scout_v2/
ls
cd scout_ros2/
ls
rm -rf scout_control/
ls
cd ..
ls
cd ../..
ls
colcon build --symlink-install
rm -rf build/ install/ log/
colcon build --symlink-install
clear
colcon build --symlink-install
clear
ls
clear
vsc
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_navigation2 navigation.launch.py 
bash
cd shell_scripts/
ls
nano uni_launch.nodes.sh 
rviz2
ls
cd dev_ws/
ls
colcon build --symlink-install
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
cd dev_ws/
colcon build --symlink-install
clear
ls
clear
colcon build --symlink-install
tmux
ls
nano ~/.bashrc 
bash
ls
cd dev_ws/
rm -rf build/ install/ log/
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/
cmake ..
make install
cd
cd dev_ws/
c\olcon build --symlink-install
clear
ls
cd dev_ws/
vsc
cat ~/.tmux.conf 
bash
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
rviz2
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
rviz2
ls
rm -rf ram.sh 
ls
nano launch_exploration.sh
vsc
tmux
ls
sudo chmod +x launch_exploration.sh 
./launch_exploration.sh 
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
cd dev_ws && vsc
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
rviz2
cd dev_ws/
colcon build --symlink-install
clear
nano ~/.tmux.conf 
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd dev_ws && vsc
colcon build --symlink-install
vsc
pidof code
kill -9 $(pidof code)
vsc
colcon build --symlink-install
vsc
colcon build --symlink-install
vsc
colcon build --symlink-install
clear
colcon build --symlink-install
pidof code
kill -9 $(pidof code)
vsc
colcon build --symlink-install
cd build/
ls
cd frontier_
cd frontier_exploration/
ls
./logger_test 
clear
./logger_test 
clear
./logger_test 
clear
./logger_test 
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
apt-get install gdb*
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
kill -9 %1
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ls
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ls
cd de
cd dev_ws/
colcon build --symlink-install
ls
cd dev_ws/
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ls
cd
cd de
cd dev_ws/
colcon build --symlink-install
ls
colcon build --symlink-install
cd dev_ws/
vsc
cd dev_ws && vsc
rviz2
apt-get install ros-humble-grid-map-rviz-plugins
apt-get install ros-humble-grid-map-rviz*
apt-get update --fix-missing
rviz2
apt-get install ros-humble-grid-map-rviz*
rviz2
rviz2
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0k
./launch_exploration.sh 
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ls
cat defaultFilename_20240923_114456_853.csv | grep fukk
cat defaultFilename_20240923_114456_853.csv | grep full
cat defaultFilename_20240923_114456_853.csv | grep construction
cat defaultFilename_20240923_114456_853.csv
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ros2 topic echo /exploration_path_followed 
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
apt-get install ros-humble-grid-map
apt-get install ros-humble-octomap
apt-get install ros-humble-octomap-rv*
apt-get update --fix-missing
apt-get install ros-humble-octomap-rv*
apt-get install ros-humble-octomap
apt-get install ros-humble-octomap*
rviz2 -d dev_ws/frontier_exploration.rviz 
rviz2
rviz2 -d dev_ws/frontier_exploration.rviz 
bash
cd dev_ws && vsc
ros2 topic echo /robot_0/cmd_vel_nav 
colcon build --symlink-install
ros2 topic info /robot_0/cmd_vel_nav 
colcon build --symlink-install
ros2 topic echo /robot_0/connecting_cells
colcon build --symlink-install
ros2 topic echo /robot_0/connecting_cells
colcon build --symlink-install
ros2 topic hz /robot_0/lidar
ros2 topic hz /robot_0/lidar/points 
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
apt-get install gdb
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
apt-get install gdbserver
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
apt-get install gdbserver
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ls
rm -rf occupancy_grid_count_202409*
ls
rm -rf defaultFilename_202409*
ls
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
./launch_exploration.sh 
rviz2
cd dev_ws && vsc
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd dev_ws/
svc
vsc
colcon build --symlink-install
cd
./launch_exploration.sh 
/bin/python3 /root/comparisions/compnbvp/cave_world/compare.py
/bin/python3 /root/comparisions/compgbp2/cave_world/compare.py
/bin/python3 /root/comparisions/compfslam1/cave_world/compare.py
/bin/python3 /root/comparisions/compnbvp/cave_world/compare.py
/bin/python3 /root/comparisions/compgbp2/cave_world/compare.py
/bin/python3 /root/comparisions/compfslam1/cave_world/compare.py
/bin/python3 /root/comparisions/compare_cave.py
/bin/python3 /root/comparisions/compfslam2/cave_world/compare.py
/bin/python3 /root/comparisions/compare_cave.py
/bin/python3 /root/comparisions/compgbp2/cave_world/compare.py
/bin/python3 /root/comparisions/compgbp2/cave_world/cleanup.py
ls
rm -rf rt_cave_gbp2.csv 
pwd
cd /root/comparisions/compgbp2/cave_world/cleanup.py
cd /root/comparisions/compgbp2/cave_world/
ls
python3 cleanup.py 
/bin/python3 /root/comparisions/compgbp2/marsyard/compare.py
cd ..
ls
cd marsyard/
python3 cleanup.py 
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd dev_ws && vsc
colcon build --symlink-install
clear
cleaR
CLEAR
clear
colcon build --symlink-install
clear
colcon build --symlink-install
clear
colcon build --symlink-install
pidof code
kill -9 $(pidof code)
vsc
colcon build --symlink-install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
colcon build --symlink-install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
colcon build --symlink-install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
colcon build --symlink-install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
colcon build --symlink-install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 node list
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 node list
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 node list
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 topic echo /robot_0/cmd_vel_nav
ros2 topic echo /robot_0/cmd_vel
rviz2
cd dev_ws/
vsc
ls
cd
ls
./launch_exploration.sh 
rviz2
cd dev_ws && vsc
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd dev_ws && vsc
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
ls
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
rviz2 -d dev_ws/frontier_exploration.rviz
cd dev_ws && vsc
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
clEAR
clear
ls
./launch_exploration.sh 
ls
cd dev_ws/
ls
cd
./launch_exploration.sh 
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
rviz2 -d dev_ws/frontier_exploration.rviz
cd dev_ws && vsc
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
./launch_
./launch_exploration.sh 
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
rviz2 -d dev_ws/frontier_exploration.rviz
cd dev_ws && vsc
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
cd dev_ws && vsc
source /root/trav_ws/setup.bash
cd
source /root/trav_ws/setup.bash
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
source trav_ws/install/setup.bash
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
rviz2 -d dev_ws/frontier_exploration.rviz
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ls
./launch_
./launch_exploration.sh 
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
./launch_exploration.sh 
rviz2 -d dev_ws/frontier_exploration.rviz
cd dev_ws && vsc
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd dev_ws && vsc
cd
cd colcon_ws/
vsc
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
rviz2 -d dev_ws/frontier_exploration.rviz
rviz2
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
cd
cd colcon_ws/
vsc
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
rm -rf /home/orb/ORB_SLAM3/build
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
ros2 node list
./launch_exploration.sh 
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
apt-get install ros-humble-navigation2*
apt-get install ros-humble-nav2*
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
cd
python3 colcon_ws/src/orb_slam3_ros2_wrapper/scripts/get_landmarks_for_pose.py 
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
python3 colcon_ws/src/orb_slam3_ros2_wrapper/scripts/get_landmarks_for_pose.py 
cd
python3 colcon_ws/src/orb_slam3_ros2_wrapper/scripts/get_landmarks_for_pose.py 
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
rviz2 -d dev_ws/frontier_exploration.rviz
rviz2
rviz2 -d dev_ws/frontier_exploration.rviz
ros2 topic list | grep smoother
ros2 node info smoother
ros2 node list | grep smoother
clear
ros2 node list | grep smoother
ros2 node info /robot_0/smoother_server 
ros2 topic echo /robot_0/global_costmap/costmap
ros2 topic echo /robot_0/global_costmap/costmap_raw
clear
cd dev_ws && vsc
vsc
bash
ros2 topic list 
cd dev_ws/
colcon build --symlink-install
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 topic list | grep global_costmap
bash
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
clear
ls
./launch_
./launch_exploration.sh 
rviz2 -d dev_ws/frontier_exploration.rviz
cd dev_ws && vsc
vsc
ros2 topic info /robot_0/global_costmap/costmap_raw
ros2 topic info -v /robot_0/global_costmap/costmap_raw
ros2 topic info -v /robot_0/traversability_thresholded 
clear
ros2 topic info -v /robot_0/traversability_thresholded 
clear
ros2 topic info -v /robot_0/traversability_thresholded 
ros2 topic echo /robot_0/traversability_thresholded
ros2 topic echo /robot_0/traversability_thresholded --field header
ros2 topic echo /robot_0/global_traversability_map --field header
ros2 topic echo /robot_0/odom
ros2 topic echo /robot_0/depth_camera
ros2 topic echo /robot_0/ground_truth_pose --field header
ros2 topic echo /robot_0/global_traversability_map --field header
ros2 topic echo /robot_0/traversability_thresholded --field header
ros2 topic info -v /robot_0/traversability_thresholded
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 topic echo /robot_0/traversability_thresholded 
nano ~/.bashrc 
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cclear
clear
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
ros2 topic echo /explore_costmap
ros2 topic echo /robot_0/explore_costmap/costmap
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
ls
clear
s
ls
./launch_
./launch_exploration.sh 
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd dev_ws && vsc
rviz2 -d dev_ws/frontier_exploration.rviz
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
clear
ls
./launch_exploration.sh 
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd dev_ws && vsc
rviz2 -d dev_ws/frontier_exploration.rviz
ls
./launch_exploration.sh 
ls
cd dev_ws && vsc
rviz2 -d dev_ws/frontier_exploration.rviz
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
./launch_exploration.sh 
cd dev_ws && vsc
ros2 topic echo /robot_0/explore_costmap/costmap
ros2 topic echo /robot_0/explore_costmap/costmap_raw
apt remove ros-humble-nav*
git clone -b humble https://github.com/ros-navigation/navigation2
colcon build --symlink-install
apt-get install ros-humble-nav-msgs
git clone -b humble https://github.com/ros-navigation/navigation2
colcon build --symlink-install
apt-get install ros-humble-test-msgs
colcon build --symlink-install
apt-get install ros-humble-rviz*
apt-get install ros-humble-rviz-de*
cd navigation2/
ls
git branch
mv nav2_costmap_2d/ ../
ls
cd ..
sl
ls
rm -rf navigation2/
ls
apt-get install ros-humble-navigaion2
apt-get install ros-humble-navigat*
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
rviz2 -d dev_ws/frontier_exploration.rviz
clear
ls
cd dev_ws/
rviz2 -d dev_ws/frontier_exploration.rviz
apt remove ros-humble-nav2*
apt remove ros-humble-navigation2
apt get install -y ros-humble-navigation2
apt-get install -y ros-humble-navigation2
cd
ls
./launch_exploration.sh 
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd dev_ws && vsc
colcon build --symlink-install
rviz2 -d dev_ws/frontier_exploration.rviz
clear
ls
apt-get install ros-humble-nav2*
clear
ls
cd dev_ws/
ls
colcon build --symlink-install
rm -rf build/ install/ lo
rm -rf build/ install/ log/
colcon build --symlink-install
cd
./launch_exploration.sh 
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
rviz2 -d dev_ws/frontier_exploration.rviz
cd dev_ws && vsc
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
clear
ls
rm -rf occupancy_grid_count_20241014_1*
ls
rm -rf core.278813 
git status
clear
ls
clear
ls
./launch_exploration.sh 
vsc
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd dev_ws && vsc
colcon build --symlink-install
rviz2 -d dev_ws/frontier_exploration.rviz
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
cd dev_ws/
vsc
clear
ls
cd
clear
ls
./launch_
./launch_exploration.sh 
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
rviz2 -d dev_ws/frontier_exploration.rviz
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
rviz2 -d dev_ws/frontier_exploration.rviz
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
rviz2 -d dev_ws/frontier_exploration.rviz
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
rviz2 -d dev_ws/frontier_exploration.rviz
clear
vsc
sudo chmod +x launch_*
ls
./launch_active_slam.sh 
ls
rm -rf dev_ws/install/ dev_ws/build/ dev_ws/log/
./launch_active_slam.sh 
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
rviz2 -d dev_ws/frontier_exploration.rviz
rviz2
ls
./launch_active_slam.sh 
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
rviz2 -d /root/frontier_exploration.rviz
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
vsc
./launch_active_slam.sh 
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
cd dev_ws/
vsc
rviz2 -d /root/frontier_exploration.rviz
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
rviz2 -d /root/frontier_exploration.rviz
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
ros2 node list
cd /home/traversability/traversability_mapping/ && ./build.sh && cd /home/orb/ORB_SLAM3/ && ./build.sh && cd /root/colcon_ws/ && colcon build --symlink-install && cd /root/trav_ws/ && colcon build --symlink-install
ros2 launch robot_navigation2 navigation.launch.py robot_namespace:=robot_0
ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py 
rviz2 -d dev_ws/frontier_exploration.rviz
cd && source colcon_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py
rviz2 -d /root/frontier_exploration.rviz
ros2 launch robot_navigation2 navigation_fast.launch.py robot_namespace:=robot_0
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
ls
./launch_active_slam.sh 
./launch_exploration.sh 
cd && source colcon_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
ros2 launch robot_navigation2 navigation_fast.launch.py robot_namespace:=robot_0
rviz2 -d /root/frontier_exploration.rviz
clear
ls
./launch_exploration.sh 
ls
sudo poweroff
ls
sudo chmod +x build_packages.sh 
./build_packages.sh 
ls
./build_packages.sh && exit
./build_packages.sh && exit # run once inside the container
ls
ros2 launch robot_navigation2 navigation_fast.launch.py robot_namespace:=robot_0
rviz2 -d /root/frontier_exploration.rviz
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
./launch_exploration.sh
ros2 launch robot_navigation2 navigation_fast.launch.py robot_namespace:=robot_0
ls
cd sr
cd dev_ws/
lsc
ls
cd src/
s
ls
rm -rf nav2_costmap_2d/
git status
ls
cd ..
ls
rm -rf build/nav2_costmap_2d/ install/nav2_costmap_2d/
cd && source colcon_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py
rviz2 -d /root/frontier_exploration.rviz
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
./launch_exploration.sh 
rviz2 -d /root/frontier_exploration.rviz
ros2 launch robot_navigation2 navigation_fast.launch.py robot_namespace:=robot_0
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
rviz2 -d /root/frontier_exploration.rviz
ros2 launch robot_navigation2 navigation_fast.launch.py robot_namespace:=robot_0
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py
./launch_exploration.sh 
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
rviz2 -d /root/frontier_exploration.rviz
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
./launch_active_slam.sh 
./build_packages.sh && exit
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
rviz2 -d /root/frontier_exploration.rviz
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
./launch_active_slam.sh 
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
rviz2 -d /root/frontier_exploration.rviz
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
rviz2 -d /root/frontier_exploration.rviz
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
rviz2 -d /root/frontier_exploration.rviz
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
rviz2 -d /root/frontier_exploration.rviz
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
ls
rm -rf dev_ws/install/ dev_ws/build/
./build_packages.sh 
./launch_active_slam.sh 
./build_packages.sh 
./launch_active_slam.sh 
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
ros2 launch robot_navigation2 navigation_slow.launch.py robot_namespace:=robot_0
ros2 launch frontier_exploration active_slam.launch.py robot_namespace:=robot_0
rviz2 -d /root/frontier_exploration.rviz
cd && source colcon_ws/install/setup.bash && ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py
./build_packages.sh 
./launch_active_slam.sh 
cd dev_ws/
colcon build --symlink-install
cd /root/dev_ws/src/frontier_exploration/frontier_multirobot_allocator/build/ && cmake .. && make install
cd
cd dev_ws/
colcon build --symlink-install
clear
colcon build --symlink-install
clear
cledar
clear
colcon build --symlink-install
clear
colcon build --symlink-install
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
ls
cd dev_ws/
vsc
cd src/frontier_exploration/
ls
vsc
./build_packages.sh 
ls
ros2 topic list
export ROS_DOMAIN_ID=26
ros2 topic list
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
cd && source colcon_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py
ros2 launch robot_navigation2 navigation_fast.launch.py robot_namespace:=robot_0
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
rviz2 -d /root/frontier_exploration.rviz
cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py
ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0
export ROS_DOMAIN_ID=26
ros2 launch frontier_exploration exploration.launch.py
echo $RMW_IMPLEMENTATION 
nano ~/.bashrc 
nano ~/.ros/cyclonedds.xml 
ros2 launch frontier_exploration exploration.launch.py
ros2 launch robot_navigation2 navigation_fast.launch.py robot_namespace:=robot_0
rviz2 -d /root/frontier_exploration.rviz
cd && source colcon_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py
ros2 topic list
nano ~/.bashrc 
cd .ros/
ls
nano cyclonedds.xml 
cd
export ROS_DOMAIN_ID=26
ros2 topic list
apt-get install ros-humble-rmw-cyclonedds-cpp
nano ~/.bashrc 
nano ~/.ros/cyclonedds.xml 
ls
ros2 topic list
echo $ROS_LOCALHOST_ONLY 
export ROS_LOCALHOST_ONLY=0
ros2 topic list
nano ~/.bashrc 
bash
