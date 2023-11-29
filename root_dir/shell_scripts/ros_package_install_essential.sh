#!/bin/bash

apt-get update
apt-get upgrade
apt update --fix-missing

apt-get install ros-humble-turtlesim -y
apt-get install ros-humble-rqt ros-humble-rqt-common-plugins -y
apt-get install ros-humble-gazebo-ros-pkgs -y
apt-get install ros-humble-rviz2 -y

#Cartographer
apt install ros-humble-cartographer -y
apt install ros-humble-cartographer-ros -y

#rosdep
rosdep init
rosdep update
. /opt/ros/humble/setup.sh
apt-get install python3-rosdep -y
echo '151.101.84.133 raw.githubusercontent.com' | tee -a /etc/hosts
cat /etc/hosts
rosdep init
rosdep update
rm -rf /var/lib/apt/lists/*