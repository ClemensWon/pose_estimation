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
  rostopic pubspawn_trigger std_msgs/Empty "{}
  rostopic echo /object_spawned
  rostopic pubdelete_object your_package/DeleteObject "object_id: '1234-5678-90ab-cdef'"

https://www.youtube.com/watch?v=O0aZ0XFEYbU&list=PLJOHOcnvyOr3OqdanYQZIf7Rnga3VLmJA&index=5
