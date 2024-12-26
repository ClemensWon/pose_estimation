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
- Object Handling:
  rostopic pub spawn_trigger std_msgs/Empty "{}"
  rostopic echo /object_spawned
  rostopic pub delete_object your_package/DeleteObject "object_id: '1234-5678-90ab-cdef'"

### To test moveit
In folder /catkin_ws:
- roslaunch pose_estimation spawn_ur5_moveit.launch
- send some position values to moveit node:
  - rostopic pub /joint_values std_msgs/Float64MultiArray "data: [0.0, -1.57, 0.0, 0.0, 0.0, 1.57]"


### To snap a photo
In folder /catkin_ws:
- roslaunch pose_estimation spawn_ur5_moveit.launch
- rostopic pub /take_picture std_msgs/String "data: 'object1'"


https://www.youtube.com/watch?v=O0aZ0XFEYbU&list=PLJOHOcnvyOr3OqdanYQZIf7Rnga3VLmJA&index=5
