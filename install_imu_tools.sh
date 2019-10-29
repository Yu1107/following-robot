#!/bin/bash

# ref: https://github.com/ccny-ros-pkg/imu_tools

# Make sure you have git installed:
sudo apt-get install git-core

# get ROS version

echo "----------------------------------"
echo "please enter your ros version:"
echo "----------------------------------"
name=$(rosversion -d)
echo $name

#echo "----------------------------------"
#echo "please enter your ros version:"
#echo "----------------------------------"
#select var in indigo jade kinetic lunar melodic;
#do
#    break
#done
#echo "You have selected $var"
#echo "----------------------------------"

# Download the stack from our repository into your catkin workspace
echo "請輸入install workspace ex:catkin_ws"
read workspace

cd ~/$workspace/src
git clone -b $var https://github.com/ccny-ros-pkg/imu_tools.git

# Install any dependencies using rosdep.
rosdep install imu_tools

# Compile the stack:
cd ..
catkin_make

echo "done"
