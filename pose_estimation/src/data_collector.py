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
from pose_estimation.srv import SpawnObject, DeleteObject, MoveJoints, SaveImage


# converting between Pose and transformation matrices
def pose_to_matrix(pose):
    """
    Convert geometry_msgs/Pose to a 4x4 transformation matrix.
    """
    trans = (pose.position.x, pose.position.y, pose.position.z)
    rot   = (pose.orientation.x, pose.orientation.y, 
             pose.orientation.z, pose.orientation.w)

    mat_trans = tft.translation_matrix(trans)
    mat_rot   = tft.quaternion_matrix(rot)
    return tft.concatenate_matrices(mat_trans, mat_rot)

def matrix_to_pose(mat):
    """
    Convert a 4x4 transformation matrix to geometry_msgs/Pose.
    """
    trans = tft.translation_from_matrix(mat)
    quat  = tft.quaternion_from_matrix(mat)

    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def compute_relative_transform(pose_a, pose_b):
    """
    Compute T_A_B = (T_world_A)^{-1} * (T_world_B)
    given geometry_msgs/Pose for A (tool0) and B (object) in the world frame.
    
    Returns a geometry_msgs/Pose representing the transform from A->B.
    """
    mat_a_world = pose_to_matrix(pose_a)  # T_world_A
    mat_b_world = pose_to_matrix(pose_b)  # T_world_B

    # Invert T_world_A -> T_A_world
    mat_world_a = mat_a_world
    mat_a_world_inv = tft.inverse_matrix(mat_world_a)

    # T_A_B = T_A_world^-1 * T_world_B
    mat_a_b = tft.concatenate_matrices(mat_a_world_inv, mat_b_world)
    return matrix_to_pose(mat_a_b)


class DataCollector:
    def __init__(self):
        rospy.init_node("data_collector")

        # Read configuration for data generation
        config = rospy.get_param('/generate_dataset')
        self.iterations = config['iterations']
        self.robot_poses = config['poses']

        # Wait for Gazebo to unpause
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        time.sleep(5)
        self.unpause()

        # Wait for required services
        rospy.wait_for_service('spawn_object')
        rospy.wait_for_service('delete_object')
        rospy.wait_for_service('move_joints')
        rospy.wait_for_service('save_image')

        # NEW: Wait for /gazebo/get_model_state to retrieve object world pose
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Create service proxies
        self.spawn_service      = rospy.ServiceProxy('spawn_object', SpawnObject)
        self.move_joints        = rospy.ServiceProxy('move_joints', MoveJoints)
        self.delete_service     = rospy.ServiceProxy('delete_object', DeleteObject)
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

        # Store dataset entries
        self.dataset = []
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


    def delete_object(self, object_id):
        response = self.delete_service(object_id=object_id)
        if response.success:
            rospy.loginfo(f"Deleted object with ID: {object_id}")
        else:
            rospy.logerr(f"Failed to delete object: {response.message}")


    def take_picture(self, image_name):
        rospy.loginfo(f"Capturing image: {image_name}")
        self.current_image_name = image_name
        response = self.save_image_service(image_name)
        return response


    def save_data(self):
        dataset_file = os.path.join(self.dataset_folder, "dataset.json")
        with open(dataset_file, "w") as f:
            json.dump(self.dataset, f, indent=4)
        rospy.loginfo(f"Dataset saved to {dataset_file}")


    # retrieve the object's pose in the world frame using /gazebo/get_model_state
    def get_object_world_pose(self, model_name):
        """
        Get the Pose of a spawned model in the 'world' frame from Gazebo.
        Returns geometry_msgs/Pose or None on failure.
        """
        try:
            resp = self.get_model_state(model_name, 'world')
            return resp.pose
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call /gazebo/get_model_state failed for {model_name}: {e}")
            return None


    # retrieve the tool0 pose in the world frame using TF
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
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Could not find transform world->tool0: {e}")
            return None


    # Save the camera->object transform (tool0->object) plus other info
    def save_transformation_data(self, image_name, object_id, object_type):
        """
        1. Get the object's pose in the world via GetModelState
        2. Get the tool0 pose in the world via TF
        3. Compute tool0->object transform
        4. Store in self.dataset
        """
        # object name
        model_name = f"{object_type}_{object_id}"

        # object's world pose
        object_world_pose = self.get_object_world_pose(model_name)
        if not object_world_pose:
            rospy.logwarn(f"Could not retrieve object pose for {model_name}")
            return

        # tool0's world pose
        tool0_world_pose = self.get_tool0_world_pose()
        if not tool0_world_pose:
            rospy.logwarn("Could not retrieve tool0 world pose.")
            return

        # compute T_tool0_object
        tool0_object_pose = compute_relative_transform(tool0_world_pose, object_world_pose)

        trans = [
            tool0_object_pose.position.x,
            tool0_object_pose.position.y,
            tool0_object_pose.position.z
        ]
        rot = [
            tool0_object_pose.orientation.x,
            tool0_object_pose.orientation.y,
            tool0_object_pose.orientation.z,
            tool0_object_pose.orientation.w
        ]

        data_entry = {
            "image_name": image_name,
            "object_id": object_id,
            "object_type": object_type,
            "camera_to_object": {
                "translation": trans,
                "rotation": rot
            }
        }
        self.dataset.append(data_entry)
        self.save_data()


    def run(self):
        try:
            rospy.loginfo("Starting data collection...")

            for n in range(self.iterations):
                # 1) Spawn object
                spawn_msg = self.spawn_object()
                if spawn_msg is None:
                    continue

                object_id   = spawn_msg.object_id
                object_type = spawn_msg.object_type

                # 2) For each robot pose, move, take picture, save transform
                for i, pose in enumerate(self.robot_poses):
                    rospy.loginfo(f"Data collection cycle {i+1} for iteration {n}")

                    # a) Move robot
                    self.move_robot(pose)

                    # b) Take image
                    image_name = f"image_{n}_pose{i+1}"
                    self.take_picture(image_name)

                    # c) Save camera->object transform
                    self.save_transformation_data(image_name, object_id, object_type)

                # 3) Delete the spawned object
                self.delete_object(object_id)

            rospy.loginfo("Data collection complete.")

        except rospy.ROSInterruptException:
            rospy.loginfo("Interrupted before completion.")


if __name__ == "__main__":
    collector = DataCollector()
    collector.run()
