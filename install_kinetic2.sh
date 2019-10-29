#!/bin/bash

#ref: http://wiki.ros.org/kinetic/Installation/Ubuntu

# 1.Installation
echo 'Installation'
echo "----------------------------------"
cd
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
apt-cache search ros-kinetic
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

echo 'Installation done'
echo "----------------------------------"

# 2.Create a ROS Workspace

mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH

echo 'ROS Workspace done'
echo "----------------------------------"

