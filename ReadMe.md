## Project 5

### Installation / Initialization

#### First steps

- Start Container bash .....sh
- CLONE ONCE:  
   Folder catkin_ws/src:  
   git clone https://github.com/ros-industrial/universal_robot  
   git clone https://github.com/ClemensWon/PoseEstimation

#### Basic Install

MANUALLY

- In Folder /catkin_ws:  
   sudo apt-get update  
   sudo apt-get install ros-noetic-gazebo-ros ros-noetic-moveit ros-noetic-tf-conversions ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager
  sudo apt-get install ros-noetic-joint-state-publisher-gui

OR AUTOMATIC

- In Folder /catkin_ws:
  bash ./src/pose_estimation/start_up.sh

#### Make

- In Folder /catkin_ws:
  catkin_make  
  source devel/setup.bash

### Test moveit

- roslaunch pose_estimation spawn_main.launch
- send some position values to moveit node:
  - rostopic pub /joint_values std_msgs/Float64MultiArray "data: [0.0, -1.57, 0.0, 0.0, 0.0, 1.57]"

### Open Close Gripper

In folder /catkin_ws:

- roslaunch pose_estimation spawn_main.launch
- send some position values to moveit node:
  OPEN:
  rosrun your_package_name gripper.py 0.0
  CLOSE:
  rosrun your_package_name gripper.py 0.75

### Go and Grab

In folder /catkin_ws:

rosrun pose_estimation move_end_effector.py <x> <y> <z> [roll] [pitch] [yaw] [gripper_pos]

<x>: Target X-coordinate (meters)
<y>: Target Y-coordinate (meters)
<z>: Target Z-coordinate (meters)
[roll] (Optional): Roll angle in radians (default=0.0)
[pitch] (Optional): Pitch angle in radians (default=0.0)
[yaw] (Optional): Yaw angle in radians (default=0.0)
[gripper_pos] (Optional): Gripper position (e.g., 0.0 = closed, 0.04 = open). If omitted, the gripper does not move.

- With Orientation & Gripper Command
  rosrun pose_estimation move_end_effector.py 0.4 0.2 0.3 0.0 1.57 0.0 0.0

### Snap a photo

In folder /catkin_ws:

- roslaunch pose_estimation spawn_main.launch
  rostopic pub /take_picture std_msgs/String "data: 'object1'"

### Spawn object

- Object Handling:
  rostopic pub spawn_trigger std_msgs/Empty "{}
  rostopic echo /object_spawned
  rostopic pubdelete_object your_package/DeleteObject "object_id: '1234-5678-90ab-cdef'"

## Directory Structure

/pose_estimation (main directory)
../config
../launch
../maps
../msg
../src
../urdf

/robotiq_description (gripper directory)
/moveitconfig (moveit config directory)
