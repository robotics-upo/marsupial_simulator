#!/bin/bash

read -p "Installation PACKAGES Dependencies for Marsupial Simulator. Press a ENTER to contine. (CTRL+C) to cancel"


# 1. Ignition gazebo fortress (source https://gazebosim.org/docs/fortress/install_ubuntu)

# 1a. necessary packages
sudo apt-get update
sudo apt-get install lsb-release wget gnupg -y

# 2a. Install IGN fortress
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress -y
sudo apt-get install ros-noetic-joy


# Install ROS-IGN bridge (noetic)
cd ~/marsupial_ws/src      # We assume that the catkin ws is in that folder
git clone -b noetic https://github.com/gazebosim/ros_gz.git
export IGNITION_VERSION=fortress
cd ..
source devel/setup.bash
catkin_make

