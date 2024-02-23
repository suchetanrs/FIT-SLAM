#!/bin/bash

# Function to terminate all background processes and exit the script
function cleanup_and_exit {
    echo "Ctrl+C detected. Terminating all background processes..."
    kill $(jobs -p)
    exit 1
}

# Trap Ctrl+C signal and call the cleanup_and_exit function
trap cleanup_and_exit INT

export SCOUT_NAMESPACE="scout_2"
export SCOUT_X="1.0"
export SCOUT_Y="1.0"
mkdir /home/$SCOUT_NAMESPACE
ros2 launch frontier_exploration exploration.launch.py &

export SCOUT_NAMESPACE="scout_1"
export SCOUT_X="-8.5"
export SCOUT_Y="7.5"
mkdir /home/$SCOUT_NAMESPACE
ros2 launch frontier_exploration exploration.launch.py &

wait
