- Start Container bash .....sh
- CLONE ONCE:  
   Folder catkin_ws/src:  
   git clone https://github.com/ros-industrial/universal_robot  
   git clone https://github.com/ClemensWon/PoseEstimation
- In Folder /catkin_ws:  
   sudo apt-get update  
   sudo apt-get install ros-noetic-gazebo-ros ros-noetic-moveit ros-noetic-tf-conversions ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager
  sudo apt-get install ros-noetic-joint-state-publisher-gui
- In Folder /catkin_ws:
  catkin_make  
  source devel/setup.bash
  roslaunch PoseEstimation spawn_ur5_cam.launch


### Object Handling:
  rostopic pub spawn_trigger std_msgs/Empty "{}"
  rostopic echo /object_spawned
  rostopic pub delete_object pose_estimation/DeleteObject "object_id: 'e3b235e0-1d86-4aa9-b603-d40115f3e2e8'"

### To test moveit
In folder /catkin_ws:
- roslaunch pose_estimation spawn_ur5_moveit.launch
- send some position values to moveit node:
  - rostopic pub /joint_values std_msgs/Float64MultiArray "data: [0.0, -1.57, 0.0, 0.0, 0.0, 1.57]"


### To snap a photo
In folder /catkin_ws:
- roslaunch pose_estimation spawn_ur5_moveit.launch
- rostopic pub /take_picture std_msgs/String "data: 'object1'"


### To run the data collector script
In folder /catkin_ws:
- roslaunch pose_estimation spawn_ur5_moveit.launch
- rosrun pose_estimation data_collector.py

https://www.youtube.com/watch?v=O0aZ0XFEYbU&list=PLJOHOcnvyOr3OqdanYQZIf7Rnga3VLmJA&index=5
