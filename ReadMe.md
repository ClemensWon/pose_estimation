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
3.) in folder catkin_ws/:
3.1) bash ./src/pose_estimation/start_up.sh
3.2) bash source devel/setup.bash

#### Object Handling:

rostopic pub spawn_trigger std_msgs/Empty "{}"
rostopic echo /object_spawned
rostopic pub delete_object pose_estimation/DeleteObject "object_id: 'e3b235e0-1d86-4aa9-b603-d40115f3e2e8'"

#### Test moveit

### Test moveit

- roslaunch pose_estimation spawn_main.launch
- send some position values to moveit node:
- rosservice call /move_joints "joint_values: [0.0, -1.57, 0.0, 0.0, 0.0, 1.57]"

### Open Close Gripper

In folder /catkin_ws:

- roslaunch pose_estimation spawn_main.launch
- send some position values to moveit node:
  OPEN:
  rosrun pose_estimation gripper.py 0.0
  CLOSE:
  rosrun pose_estimation gripper.py 0.75

### Snap a photo

In folder /catkin_ws:

- roslaunch pose_estimation spawn_main.launch
  rostopic pub /take_picture std_msgs/String "data: 'object1'"

#### Data Generation

In folder /catkin_ws:

- roslaunch pose_estimation generate_dataset.launch
