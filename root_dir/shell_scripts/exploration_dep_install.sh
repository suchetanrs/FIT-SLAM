#!/bin/bash

apt-get update
apt-get upgrade
apt update --fix-missing

apt-get install -y ros-humble-navigation2
apt-get install -y ros-humble-nav2-costmap-2d
apt-get install -y ros-humble-nav2-msgs
apt-get install -y ros-humble-nav2-util