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
2.1) git clone https://github.com/ClemensWon/pose_estimation.git (only once)
2.2) the cloned folder name needs to be pose_estimation (catkin_ws/src/pose_estimation)
3.) In folder catkin_ws/:
3.1) bash ./src/pose_estimation/start_up.sh
3.2) we install all plugins, python modules and run catkin_make (this can take a while because we also install torch for prediction)
3.3) bash source devel/setup.bash
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

Action to collect data for model training

- In folder /catkin_ws:
- roslaunch pose_estimation generate_dataset.launch

important files are: generate_dataset.launch / data_collector.py

#### Pick and place

Action for pick and place action

- In folder /catkin_ws:
- roslaunch pose_estimation spawn_pick_up.launch

important files are: spawn_pick_up.launch / pick_and_place.py

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
