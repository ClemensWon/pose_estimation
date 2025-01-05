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
from pose_estimation.srv import SpawnObject, GripperControl, MoveJoints, SaveImage, MoveToPose



class pick_and_place:
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

        # Wait for /gazebo/get_model_state to retrieve object world pose
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Create service proxies
        self.spawn_service      = rospy.ServiceProxy('spawn_object', SpawnObject)
        self.move_joints        = rospy.ServiceProxy('move_joints', MoveJoints)
        self.save_image_service = rospy.ServiceProxy('save_image', SaveImage)

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
        
    def move_to_object(self, data_entry):
        """
        Move the robot's tool0 to align with the object's pose based on the provided data entry.
        """
        rospy.loginfo(f"Moving to object with data entry: {data_entry}")
        try:
            # Extract the translation and rotation from the data entry
            translation = data_entry["camera_to_object"]["translation"]
            rotation = data_entry["camera_to_object"]["rotation"]
            
            # Extract additional metadata (optional, for logging/debugging)
            image_name = data_entry["image_name"]
            object_id = data_entry["object_id"]
            object_type = data_entry["object_type"]

            # Create a service proxy for the move_to_pose service
            rospy.wait_for_service('move_to_pose')
            move_to_pose_service = rospy.ServiceProxy('move_to_pose', MoveToPose)

            # Call the service
            response = move_to_pose_service(
                translation=translation,
                rotation=rotation,
                image_name=image_name,
                object_id=object_id,
                object_type=object_type
            )

            if response.success:
                rospy.loginfo("Successfully moved to the object!")
            else:
                rospy.logerr(f"Failed to move to the object: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    

    def run(self):
        try:
            rospy.loginfo("Starting pick-and-place operation...")
            
            # 1) Spawn object
            spawn_response = self.spawn_object()
            if not spawn_response.success:
                rospy.logerr(f"Failed to spawn object: {spawn_response.message}")
                return
            object_id = spawn_response.object_id
            object_type = spawn_response.object_type
            rospy.loginfo(f"Spawned object with ID: {object_id}, type: {object_type}")

            
            self.move_robot(self.robot_poses[0])
            #rospy.sleep(2)
            '''
            image_name = f"pickup_image_{0}"
            self.take_picture(image_name)


            rospy.sleep(1)
            # 3) Move robot to pick pose
            rospy.loginfo("Moving to pick pose...")
            self.move_robot(self.robot_pick[0])
            rospy.sleep(1)
            # 4) Close the gripper to pick the object
            self.close_gripper()
            '''
            # Example data entry
            data_entry = {
                "image_name": "target_object",
                "object_id": "123",  # Example object ID
                "object_type": "target",  # Example object type
                "camera_to_object": {
                    "translation": [-0.19760395907289863, 0.4160319400437681, 0.03015402951701876],
                    "rotation": [0.55344705424579, -0.5006017434978055, -0.3805581669868574, 0.5461407639914998]
                }
            }

            # Call the move_to_object function with the data entry
            rospy.sleep(2)  # Optional delay if needed for synchronization
            self.move_to_object(data_entry)



            # 5) Move robot to place pose (optional)
            #rospy.loginfo("Moving to place pose...")
            #self.move_robot(self.robot_pick[1])  # Assuming the second pose is for placing

            # 6) Open the gripper to release the object
            #self.open_gripper()

            rospy.loginfo("Pick-and-place operation complete.")

        except rospy.ROSInterruptException:
            rospy.loginfo("Operation interrupted before completion.")



if __name__ == "__main__":
    collector = pick_and_place()
    collector.run()
