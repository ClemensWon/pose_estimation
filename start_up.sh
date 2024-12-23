#!/bin/bash

# Ensure the script is executed in the catkin_ws directory
if [ "$(basename $(pwd))" != "catkin_ws" ]; then
    echo "Error: This script must be run from the 'catkin_ws' directory."
    echo "Please navigate to the 'catkin_ws' directory and run the script."
    exit 1
fi

# Update package lists
echo "Updating package lists..."
sudo apt-get update

# Install dependencies
echo "Installing ROS and Gazebo dependencies..."
sudo apt-get install -y \
    ros-noetic-gazebo-ros \
    ros-noetic-moveit \
    ros-noetic-tf-conversions \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-controller-manager

# Confirm installation
echo "Installation complete. All dependencies have been installed."
