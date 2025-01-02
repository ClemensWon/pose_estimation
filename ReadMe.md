## Project 5

### Pose Estimation

#### Directory

catkin_ws/
└── src/
├── pose_estimation/ (main folder)
└── .gitignore
└── ReadMe.md
└── startup.sh (startup script)
├── pose_estimation/  
 ├── config/
├── dataset/
├── launch/  
 ├── maps/
├── msg/  
 ├── saved_images/
├── src/
├── srv/
├── urdf/
├── robotiq_description/

#### Initialization

1.) Start Container
2.) in folder catkin_ws/src/:
2.1) git clone https://github.com/ClemensWon/PoseEstimation (only once)
git clone https://github.com/ros-industrial/universal_robot (only once)
3.) in folder catkin_ws/:
3.1) bash ./src/pose_estimation/startup.sh
3.2) bash source devel/setup.bash
4.) roslaunch pose_estimation spawn_ur5_cam.launch

#### Object Handling:

rostopic pub spawn_trigger std_msgs/Empty "{}"
rostopic echo /object_spawned
rostopic pub delete_object pose_estimation/DeleteObject "object_id: 'e3b235e0-1d86-4aa9-b603-d40115f3e2e8'"

#### Test moveit

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

#### Collect Data

**manually**
In folder /catkin_ws:

- roslaunch pose_estimation spawn_ur5_moveit.launch
- rosrun pose_estimation data_collector.py

**automatically**
In folder /catkin_ws:

- roslaunch pose_estimation generate_dataset.launch
