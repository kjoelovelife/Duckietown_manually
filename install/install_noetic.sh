#!/bin/bash

ros1_distro=noetic

    # Install ROS 1
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install -y ros-$ros1_distro-desktop-full

    # Install ROS 1 build tools
    sudo apt install -y python3-rosdep python3-rosinstall-generator python3-vcstool build-essential

echo "ROS $ros1_distro installed successfully"
