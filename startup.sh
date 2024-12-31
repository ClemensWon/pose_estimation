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

# Clean the workspace
echo "Cleaning the workspace..."
rm -rf build devel

# Build the workspace
echo "Building the workspace with catkin_make..."
catkin_make

# Check if catkin_make was successful
if [ $? -ne 0 ]; then
    echo "Error: catkin_make failed. Please check the output for errors."
    exit 1
fi

# Source the setup.bash file
echo "Sourcing the workspace..."
source devel/setup.bash

# Confirm sourcing
if [ $? -eq 0 ]; then
    echo "Workspace built and sourced successfully."
    echo "You can now run ROS nodes or launch files from this workspace."
else
    echo "Error: Failed to source the workspace. Please check for errors."
    exit 1
fi
