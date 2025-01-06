#!/usr/bin/env python3

import os
import json
import time

import rospy
import rospkg

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState
import tf
import tf.transformations as tft

from std_srvs.srv import Empty
from pose_estimation.srv import SpawnObject, GripperControl, MoveJoints, SaveImage, MoveToCoordinate

class PickAndPlace:
    def __init__(self):
        rospy.init_node("pick_and_place")

        # Read configuration for data generation
        config = rospy.get_param('/pick_and_place')
        self.robot_poses = config['photo']
        self.robot_pick = config['pick']

        # Wait for Gazebo to unpause
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        time.sleep(5)
        self.unpause()

        # Wait for required services
        rospy.wait_for_service('spawn_object')
        rospy.wait_for_service('move_joints')
        rospy.wait_for_service('save_image')
        rospy.wait_for_service('move_to_coordinate')
        
        # Wait for /gazebo/get_model_state to retrieve object world pose
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Create service proxies
        self.spawn_service      = rospy.ServiceProxy('spawn_object', SpawnObject)
        self.move_joints        = rospy.ServiceProxy('move_joints', MoveJoints)
        self.save_image_service = rospy.ServiceProxy('save_image', SaveImage)
        self.move_to_coordinate = rospy.ServiceProxy('move_to_coordinate', MoveToCoordinate)
        
        # TF listener for tool0 -> world transform
        self.tf_listener = tf.TransformListener()

        # Get package path and set dataset folder
        rospack = rospkg.RosPack()
        package_name = rospy.get_param('~package_name', 'pose_estimation')
        package_path = rospack.get_path(package_name)
        self.dataset_folder = os.path.join(package_path, 'dataset')
        os.makedirs(self.dataset_folder, exist_ok=True)

        rospy.loginfo(f"Dataset will be saved in: {self.dataset_folder}")

        self.current_image_name = None

    def move_robot(self, target_pose):
        rospy.loginfo(f"Moving robot to pose: {target_pose}")
        response = self.move_joints(target_pose)
        # Potentially check response for success/failure


            
    def spawn_object(self):
        response = self.spawn_service()
        if response.success:
            rospy.loginfo(f"Spawned object with ID: {response.object_id}")
            return response
        else:
            rospy.logerr(f"Failed to spawn object: {response.message}")
            return None

    def open_gripper(self):
        rospy.loginfo("Opening the gripper...")
        rospy.wait_for_service('gripper_control')
        try:
            gripper_control = rospy.ServiceProxy('gripper_control', GripperControl)
            response = gripper_control(position=0.0)  # Open position
            if response.success:
                rospy.loginfo(f"Gripper opened successfully: {response.message}")
            else:
                rospy.logerr(f"Failed to open gripper: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")

    def close_gripper(self):
        rospy.loginfo("Closing the gripper...")
        rospy.wait_for_service('gripper_control')
        try:
            gripper_control = rospy.ServiceProxy('gripper_control', GripperControl)
            response = gripper_control(position=0.75)  # Closed position
            if response.success:
                rospy.loginfo(f"Gripper closed successfully: {response.message}")
            else:
                rospy.logerr(f"Failed to close gripper: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")

    def take_picture(self, image_name):
        rospy.loginfo(f"Capturing image: {image_name}")
        self.current_image_name = image_name
        response = self.save_image_service(image_name)
        return response
        
    def move_robot_to_coordinate(self, target_coordinate):  # Added 'self'
        rospy.loginfo(f"Moving robot to coordinate: {target_coordinate}")
        response = self.move_to_coordinate(target_coordinate)  # Use keyword argument

    def run(self):
        try:
            rospy.loginfo("Starting pick-and-place operation...")
            
            # 2) Move to initial pose
            self.move_robot(self.robot_poses[0])
            
            # 1) Spawn object
            spawn_response = self.spawn_object()
            if not spawn_response or not spawn_response.success:
                rospy.logerr(f"Failed to spawn object: {spawn_response.message if spawn_response else 'No response'}")
                return
            object_id = spawn_response.object_id
            object_type = spawn_response.object_type
            rospy.loginfo(f"Spawned object with ID: {object_id}, type: {object_type}")

            
            rospy.sleep(2)

            # Example data entry
            data_entry = {
                "image_name": "target_object",
                "object_id": "123",  # Example object ID
                "object_type": "target",  # Example object type
                "camera_to_object": {
                    "translation": [
                        1.326429036235808,
                        -0.30861911121918606,
                        -0.08593757955372483
                    ],
                    "rotation": [
                        0.2863415578104123,
                        -0.004779112799843402,
                        0.9580842983409861,
                        -0.007755618867172294
                    ]
                }
            }


            # Define the target coordinate
            target_coordinate = Pose()
            target_coordinate.position.x = 0.75
            target_coordinate.position.y = -0.19
            target_coordinate.position.z = 0.3
            target_coordinate.orientation.x = 0.0
            target_coordinate.orientation.y = 1.0
            target_coordinate.orientation.z = 0.0
            target_coordinate.orientation.w = 0.0

            self.move_robot_to_coordinate(target_coordinate)


            rospy.sleep(2)  # Optional delay if needed for synchronization

            rospy.loginfo("Pick-and-place operation complete.")

        except rospy.ROSInterruptException:
            rospy.loginfo("Operation interrupted before completion.")

if __name__ == "__main__":
    collector = PickAndPlace()
    collector.run()
