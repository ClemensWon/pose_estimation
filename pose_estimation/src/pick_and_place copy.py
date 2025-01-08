#!/usr/bin/env python3

import os
import json
import time
import rospy
import rospkg
import numpy as np  

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState
import tf
import tf.transformations as tft

from std_srvs.srv import Empty
from pose_estimation.srv import SpawnObject, GripperControl, MoveJoints, SaveImage, MoveToCoordinate, MoveSingleJoint


def to_homogeneous(translation, rotation):
    """
    Construct a 4x4 homogeneous transform from translation and quaternion rotation.
    translation: [x, y, z]
    rotation: [qx, qy, qz, qw]
    """
    mat = tft.quaternion_matrix(rotation)  # 4x4
    mat[0, 3] = translation[0]
    mat[1, 3] = translation[1]
    mat[2, 3] = translation[2]
    return mat

def from_homogeneous(mat):
    """
    Extract translation and quaternion rotation from a 4x4 homogeneous transform.
    returns (translation, rotation)
    translation: [x, y, z]
    rotation: [qx, qy, qz, qw]
    """
    translation = [mat[0, 3], mat[1, 3], mat[2, 3]]
    rotation = tft.quaternion_from_matrix(mat)  # returns [qx, qy, qz, qw]
    return translation, rotation

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
        #rospy.wait_for_service('move_single_joint')
        
        # Wait for /gazebo/get_model_state to retrieve object world pose
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Create service proxies
        self.spawn_service      = rospy.ServiceProxy('spawn_object', SpawnObject)
        self.move_joints        = rospy.ServiceProxy('move_joints', MoveJoints)
        self.save_image_service = rospy.ServiceProxy('save_image', SaveImage)
        self.move_to_coordinate = rospy.ServiceProxy('move_to_coordinate', MoveToCoordinate)
        #self.move_single_joint = rospy.ServiceProxy('move_single_joint', MoveSingleJoint)
        
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
        
    def move_robot_to_coordinate(self, target_coordinate):
        rospy.loginfo(f"Moving robot to coordinate: {target_coordinate}")
        response = self.move_to_coordinate(target_coordinate)
    '''
    def move_single_joint(self, joint_index, joint_value):
        """
        Calls the move_single_joint service to move a specific joint.
        """
        try:
            rospy.loginfo(f"Requesting to move joint {joint_index} to {joint_value} radians.")
            response = self.move_single_joint(joint_index, joint_value)
            if response.success:
                rospy.loginfo(response.message)
            else:
                rospy.logerr(response.message)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")
    '''
    def get_tool0_world_pose(self):
        """
        Look up the pose of 'tool0' relative to 'world' via TF.
        Returns a geometry_msgs/Pose or None if lookup fails.
        """
        try:
            self.tf_listener.waitForTransform("world", "tool0", rospy.Time(0), rospy.Duration(2.0))
            (trans, rot) = self.tf_listener.lookupTransform("world", "tool0", rospy.Time(0))
            
            pose = Pose()
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]

            rospy.loginfo(f"Tool0 Position: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
            rospy.loginfo(f"Tool0 Orientation: x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w}")

            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Could not find transform world->tool0: {e}")
            return None
        
    def calculate_world_to_object_transform(self, data_entry):
        """
        Calculates the world->object transform using the world->tool0 transform
        and the tool0->object transform from the data entry.
        """
        # 1) Get transform world->tool0
        tool0_world_pose = self.get_tool0_world_pose()
        if tool0_world_pose is None:
            rospy.logerr("Failed to get world->tool0 TF.")
            return None

        world_to_tool0_translation = [
            tool0_world_pose.position.x,
            tool0_world_pose.position.y,
            tool0_world_pose.position.z
        ]
        world_to_tool0_rotation = [
            tool0_world_pose.orientation.x,
            tool0_world_pose.orientation.y,
            tool0_world_pose.orientation.z,
            tool0_world_pose.orientation.w
        ]
        T_w_t = to_homogeneous(world_to_tool0_translation, world_to_tool0_rotation)

        # 2) tool0->object (from data_entry, ignoring the word 'camera')
        tool0_obj_translation = data_entry["camera_to_object"]["translation"]
        tool0_obj_rotation = data_entry["camera_to_object"]["rotation"]
        T_t_obj = to_homogeneous(tool0_obj_translation, tool0_obj_rotation)

        # 3) world->object = world->tool0 * tool0->object
        T_w_obj = np.dot(T_w_t, T_t_obj)
        return T_w_obj

    def run(self):
        try:
            rospy.loginfo("Starting pick-and-place operation...")
            rospy.sleep(1)
            
            # (A) Move to an initial "home" or "photo" pose
            rospy.loginfo("Moving to initial pose...")
            self.move_robot(self.robot_poses[0])

            
            rospy.sleep(2)
            
            # (B) Spawn object in the scene
            rospy.loginfo("Spawning object...")
            spawn_response = self.spawn_object()

            object_id = spawn_response.object_id
            object_type = spawn_response.object_type
            rospy.loginfo(f"Spawned object with ID: {object_id}, type: {object_type}")

            rospy.sleep(1)

            # (C) Take a picture of the spawned object
            rospy.loginfo("Taking picture of spawned object...")
            if not self.take_picture('spawned_object'):
                rospy.logerr("Failed to take a picture of the spawned object. Aborting...")
                return

            rospy.sleep(1)
            # ------------------------------------------------------------------
            # Example data entry "camera_to_object" 
            # WORKS ONLY FOR POSE 1
            # ------------------------------------------------------------------
            data_entry = {
                "image_name": "target_object",
                "object_id": "123",
                "object_type": "target",
                "camera_to_object": {
                    "translation": [
                        0.3186100257392126,
                        -0.15026389618409064,
                        -0.005904878465301938
                    ],
                    "rotation": [
                        0.3231523856786325,
                        -0.19863675699689778,
                        0.7887532332152162,
                        0.48371924862724475
                    ]
                }
            }
            

            # (D) Calculate the world->object transform
            rospy.loginfo("Calculating world->object transform...")
            T_w_obj = self.calculate_world_to_object_transform(data_entry)
            if T_w_obj is None:
                rospy.logerr("Failed to calculate world->object transform. Aborting...")
                return
            
            
            # Extract final position/orientation
            world_obj_translation, world_obj_rotation = from_homogeneous(T_w_obj)
            rospy.loginfo(f"\nFinal object coordinates in world frame:\n x={world_obj_translation[0]:.4f}, y={world_obj_translation[1]:.4f}, z={world_obj_translation[2]:.4f}")

            rospy.sleep(2)

            
            #self.move_robot(self.robot_pick[2])
            
            #rospy.sleep(2)


            # (F) Move to the object position
            rospy.loginfo("Moving to the object position...")
            target_coordinate = Pose()
            target_coordinate.position.x = world_obj_translation[0]
            target_coordinate.position.y = world_obj_translation[1]
            target_coordinate.position.z = world_obj_translation[2] - 0.3
            #target_coordinate.orientation.x = world_obj_rotation[0]
            #target_coordinate.orientation.y = world_obj_rotation[1]
            #target_coordinate.orientation.z = world_obj_rotation[2]
            #target_coordinate.orientation.w = world_obj_rotation[3]
            target_coordinate.orientation.x = 0.0
            target_coordinate.orientation.y = 0.0
            target_coordinate.orientation.z = 0.0
            target_coordinate.orientation.w = 0.0

            # Move the robot to this object pose
            self.move_robot_to_coordinate(target_coordinate)

            '''
            # Move the 4th joint to a desired value (e.g., 45 degrees in radians)
            joint_index = 3  # Joint index starts at 0, so joint 4 has index 3
            joint_value = 0.785  # 45 degrees in radians
            self.move_single_joint(joint_index, joint_value)
            '''
            rospy.sleep(4)  

            self.close_gripper()


            rospy.loginfo("Pick-and-place operation complete.")

        except rospy.ROSInterruptException:
            rospy.loginfo("Operation interrupted before completion.")

if __name__ == "__main__":
    collector = PickAndPlace()
    collector.run()
