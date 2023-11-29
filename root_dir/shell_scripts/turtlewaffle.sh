#ros2 launch scout_gazebo scout_simu.launch.py
#ros2 launch scout_navigation2 navigation_launch.py
#!/bin/bash

# Function to terminate all background processes and exit the script
function cleanup_and_exit {
    echo "Ctrl+C detected. Terminating all background processes..."
    kill $(jobs -p)
    exit 1
}

# Trap Ctrl+C signal and call the cleanup_and_exit function
trap cleanup_and_exit INT

# Command 1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
#ros2 launch turtlebot3_gazebo empty_world.launch.py &
# Command 2
ros2 launch turtlebot3_navigation2 navigation2.launch.py &
# Command 3
ros2 launch rtabmap_demos turtlebot3_rgbd.launch.py &
# Command 4
# ros2 launch nav2_bringup rviz_launch.py &
# Command 5
ros2 run frontier_exploration explore_client &
# Command 6
ros2 run frontier_exploration explore_server --ros-args --params-file /home/suchetan/dev_ws/src/frontier_exploration/frontier_exploration/params/exploration_params.yaml &

# Wait for all background processes to finish
wait
