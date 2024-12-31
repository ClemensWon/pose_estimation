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

In folder /catkin_ws:

- roslaunch pose_estimation spawn_ur5_moveit.launch
- send some position values to moveit node:
  - rostopic pub /joint_values std_msgs/Float64MultiArray "data: [0.0, -1.57, 0.0, 0.0, 0.0, 1.57]"

#### Snap a photo

In folder /catkin_ws:

- roslaunch pose_estimation spawn_ur5_moveit.launch
- rostopic pub /take_picture std_msgs/String "data: 'object1'"

#### Collect Data

**manually**
In folder /catkin_ws:

- roslaunch pose_estimation spawn_ur5_moveit.launch
- rosrun pose_estimation data_collector.py

**automatically**
In folder /catkin_ws:

- roslaunch pose_estimation generate_dataset.launch
