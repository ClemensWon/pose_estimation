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
    ros-noetic-rqt-tf-tree \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-tf2-tools \
    ros-noetic-controller-manager \
    python3-pip

# Confirm installation
echo "Installation complete. All dependencies have been installed."

# Set permissions for GPU device files
echo "Setting permissions for GPU device files..."
sudo chmod a+rw /dev/dri/*

if [ $? -eq 0 ]; then
    echo "Permissions for /dev/dri/* set successfully."
else
    echo "Failed to set permissions for /dev/dri/*. Ensure this script is run as root or with sudo."
    exit 1
fi

# Install Python modules
echo "Installing Python modules: torch, torchvision, and Pillow..."
pip3 install --upgrade pip
pip3 install torch torchvision pillow tqdm



if [ $? -eq 0 ]; then
    echo "Python modules installed successfully."
else
    echo "Error: Failed to install Python modules. Please check the output for details."
    exit 1
fi

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
    chmod +x "$PACKAGE_PATH/src/"*.py "$PACKAGE_PATH/src/services/"*.py 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "All Python scripts in pose_estimation/src and pose_estimation/src/services are now executable."
    else
        echo "No Python scripts found or an error occurred."
    fi
fi
