#!/bin/bash

#rm -rf ~/.tmux.conf
#rm -rf pTpUXK9A
#wget https://pastebin.com/raw/pTpUXK9A
#cat pTpUXK9A >> ~/.tmux.conf

cd && sudo chmod +x build_packages.sh && ./build_packages.sh && cd && source ~/.bashrc

SESSION_NAME="dusty_dev_tmux"
tmux new-session -d -s $SESSION_NAME

tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 1
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

tmux send-keys -t 0 "ros2 launch frontier_exploration exploration.launch.py robot_namespace:=robot_0" C-m

tmux send-keys -t 1 "ros2 launch robot_navigation2 navigation_fast.launch.py robot_namespace:=robot_0" C-m

tmux send-keys -t 3 "rviz2 -d /root/frontier_exploration.rviz" C-m

# tmux send-keys -t 2 "cd dev_ws && vsc" C-m

tmux send-keys -t 4 "cd && source colcon_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_traversability_mapping.launch.py" C-m
tmux send-keys -t 5 "cd && source trav_ws/install/setup.bash && ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py" C-m

# Attach to the tmux session
tmux attach-session -t $SESSION_NAME
