#!/bin/bash

# Function to terminate all background processes and exit the script
function cleanup_and_exit {
    echo "Ctrl+C detected. Terminating all background processes..."
    kill $(jobs -p)
    exit 1
}

# Trap Ctrl+C signal and call the cleanup_and_exit function
trap cleanup_and_exit INT

# Essential
ros2 launch leo_gazebo leo_simu.launch.py &
ros2 launch leo_navigation2 navigation.launch.py &
ros2 launch leo_navigation2 leo_rtab_rgbd.launch.py &
ros2 launch leo_teleop key_teleop.launch.xml &
ros2 run scout_control scout_path_planner &
ros2 run scout_control scout_path_follower --ros-args --params-file /home/suchetan/dev_ws/src/frontier_exploration/frontier_exploration/params/exploration_params.yaml &
ros2 run frontier_exploration explore_client --ros-args --params-file /home/suchetan/dev_ws/src/frontier_exploration/frontier_exploration/params/exploration_params.yaml &

# Exploration
ros2 run frontier_exploration explore_server --ros-args --params-file /home/suchetan/dev_ws/src/frontier_exploration/frontier_exploration/params/exploration_params.yaml &
ros2 run frontier_exploration traversability_server --ros-args --params-file /home/suchetan/dev_ws/src/frontier_exploration/frontier_exploration/params/exploration_params.yaml &
ros2 run frontier_exploration traversability_thresholded --ros-args --params-file /home/suchetan/dev_ws/src/frontier_exploration/frontier_exploration/params/exploration_params.yaml &
ros2 run traversability_gridmap pointcloud_transformer --ros-args --params-file /home/suchetan/dev_ws/src/frontier_exploration/frontier_exploration/params/exploration_params.yaml &
#ros2 run frontier_exploration polygon_point_publisher --ros-args --params-file /home/suchetan/dev_ws/src/frontier_exploration/frontier_exploration/params/exploration_params.yaml &

#viz
#ros2 run frontier_exploration localization_pose_sub &
#ros2 run frontier_exploration map_graph_monitor &
# ros2 run frontier_exploration scout_state_logger &
# Wait for all background processes to finish
wait
