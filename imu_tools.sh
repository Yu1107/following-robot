#!/bin/bash

# ref: https://github.com/ccny-ros-pkg/imu_tools

# Make sure you have git installed:
sudo apt-get install git-core

# get ROS version
echo "請輸入電腦ROS版本 ex:indigo (英文全名小寫)"
read rosversion

# Download the stack from our repository into your catkin workspace
echo "請輸入workspace ex:catkin_ws"
read workspace

cd ~/$workspace/src
git clone -b $rosversion https://github.com/ccny-ros-pkg/imu_tools.git

# Install any dependencies using rosdep.
rosdep install imu_tools

# Compile the stack:
cd ..
catkin_make
