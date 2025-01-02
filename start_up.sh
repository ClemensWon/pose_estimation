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

# Mark all Python scripts under pose_estimation/src as executable
echo "Marking all Python scripts in pose_estimation/src as executable..."
PACKAGE_PATH="$(rospack find pose_estimation)"
if [ -z "$PACKAGE_PATH" ]; then
    echo "Error: Could not find the 'pose_estimation' package via rospack."
    echo "Make sure the package is in your workspace and spelled correctly."
    exit 1
else
    chmod +x "$PACKAGE_PATH/src/"*.py 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "All Python scripts in pose_estimation/src are now executable."
    else
        echo "No Python scripts found or an error occurred."
    fi
fi
