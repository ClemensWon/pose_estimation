# Project 5

## Pose Estimation

#### Authors

- Georg Talasz
- Johannes Rathgeb
- Clemens Wondrak
- Milos Jovanovic
- Tim Konrad

### Initialization

1.) Start container
1.1) bash run_docker_from_hub.sh (SHARED_DIR=/home/fhtw_user/catkin_ws/src)
2.) In folder catkin_ws/src/:
2.1) git clone https://github.com/ClemensWon/PoseEstimation (only once)
3.) In folder catkin_ws/:
3.1) bash ./src/pose_estimation/start_up.sh
3.2) bash source devel/setup.bash
4.) roslaunch pose_estimation generat_dataset.launch
5.) roslaunch pose_estimation spawn_pick_up.launch (you need model weights in folder /trained_model)

#### Directory

main directories and important files

- pose_estimation
  - .gitignore
  - ReadMe.md
  - startup.sh
  - pose_estimation/
    - config/
    - dataset/
    - launch/
      - generate_dataset.launch
      - spawn_pick_up.launch
    - maps/
    - saved_images/
    - src/
      - ml/
        - trained_model/best_pose_estimation_model.pth
        - predict.py
      - services/
        - camera_snap.py
        - gripper_service.py
        - move_to_coordinate.py
        - moveit.py
        - spawn_objects.py
      - generate_dataset.py
      - pick_and_place.py
    - srv/
    - urdf/
  - robotiq_description/
  - roboticsgroup_upatras_gazebo_plugins/
  - universal_robot/

#### External Plugins

- Universal robot
  https://github.com/ros-industrial/universal_robot
- Gripper
  https://github.com/philwall3/UR5-with-Robotiq-Gripper-and-Kinect/tree/master
- Moveit
  https://www.moveit.at/

### Main Actions

#### Data Generation

In folder /catkin_ws:

- roslaunch pose_estimation generate_dataset.launch

#### Pick and place

In folder /catkin_ws:

- roslaunch pose_estimation spawn_pick_up.launch

### Services

#### Spawn Object:

rosservice call /spawn_object "{}"
rosservice call /delete_object "object_id: 'e3b235e0-1d86-4aa9-b603-d40115f3e2e8'"

#### Test moveit

- roslaunch pose_estimation spawn_main.launch
- send some position values to moveit node:
- rosservice call /move_joints "joint_values: [0.0, -1.57, 0.0, 0.0, 0.0, 1.57]"

#### Open/Close Gripper

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

#### Move single Joint

In folder /catkin_ws:
rosservice call /move_single_joint "joint_index: 5
joint_value: -1.57"
