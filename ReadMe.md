## Project 5

### Authors

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
├── saved_images/
├── src/
├── srv/
├── urdf/
├── robotiq_description/

#### Initialization

1.) Start Container
2.) in folder catkin_ws/src/:
2.1) git clone https://github.com/ClemensWon/PoseEstimation (only once)
3.) in folder catkin_ws/:
3.1) bash ./src/pose_estimation/start_up.sh
3.2) bash source devel/setup.bash

#### Object Handling:

rosservice call /spawn_object "{}"
rosservice call /delete_object "object_id: 'e3b235e0-1d86-4aa9-b603-d40115f3e2e8'"

### Services

#### Test moveit

- roslaunch pose_estimation spawn_main.launch
- send some position values to moveit node:
- rosservice call /move_joints "joint_values: [-0.2, -1.57, 0.0, 0.0, 0.0, -0.6]"

#### Open Close Gripper

In folder /catkin_ws:

- roslaunch pose_estimation spawn_main.launch
- send some position values to moveit node:
  OPEN:
  rosrun pose_estimation gripper.py 0.0
  CLOSE:
  rosrun pose_estimation gripper.py 0.75

#### Snap a photo

In folder /catkin_ws:

- roslaunch pose_estimation spawn_main.launch
  rostopic pub /take_picture std_msgs/String "data: 'object1'"

#### Move to coordinates

In folder /catkin_ws:

- roslaunch pose_estimation spawn_pick_up.launch
  rosservice call /move_to_coordinate "target_coordinate:
  position:
  x: 0.5
  y: 0.2
  z: 0.3
  orientation:
  x: 0.0
  y: 0.707
  z: 0.0
  w: 0.707"

### Actions

#### Data Generation

In folder /catkin_ws:

- roslaunch pose_estimation generate_dataset.launch

#### Pick and place

In folder /catkin_ws:

- roslaunch pose_estimation spawn_pick_up.launch
