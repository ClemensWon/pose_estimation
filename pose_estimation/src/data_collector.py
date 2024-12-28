#!/usr/bin/env python3

import os
import json
import time

import rospy
import rospkg

from std_srvs.srv import Empty

from pose_estimation.srv import SpawnObject, DeleteObject, MoveJoints, SaveImage

class DataCollector:
    def __init__(self):
        rospy.init_node("data_collector")

        config = rospy.get_param('/generate_dataset')
        self.iterations = config['iterations']
        self.robot_poses = config['poses']

        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        time.sleep(5)
        self.unpause()
        # Initialize publishers and subscribers
        rospy.wait_for_service('spawn_object')
        rospy.wait_for_service('delete_object')
        rospy.wait_for_service('move_joints')
        rospy.wait_for_service('save_image')
        

        self.spawn_service = rospy.ServiceProxy('spawn_object', SpawnObject)  
        self.move_joints = rospy.ServiceProxy('move_joints', MoveJoints)
        self.delete_service = rospy.ServiceProxy('delete_object', DeleteObject)
        self.save_image_service = rospy.ServiceProxy('save_image', SaveImage)
         
        
        

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

    def spawn_object(self):
        response = self.spawn_service()
        if response.success:
            print(f"Spawned object with ID: {response.object_id}")
            return response
        else:
            print(f"Failed to spawn object: {response.message}")

    def delete_object(self, object_id):
        response = self.delete_service(object_id=object_id)
        if response.success:
            print(f"Deleted object with ID: {object_id}")
        else:
            print(f"Failed to spawn object: {response.message}")

        

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

    def save_transformation_data(self, image_name, object_id, spawn_msg, robot_poses):
        data_entry = {
            "image_name": image_name,
            "object_id": object_id,
            "object_type": spawn_msg.object_type,
            "position": {
                "x": spawn_msg.pose.position.x,
                "y": spawn_msg.pose.position.y,
                "z": spawn_msg.pose.position.z
            },
            "orientation": {
                "x": spawn_msg.pose.orientation.x,
                "y": spawn_msg.pose.orientation.y,
                "z": spawn_msg.pose.orientation.z,
                "w": spawn_msg.pose.orientation.w
            },
            "robot_pose": robot_poses
        }
        self.dataset.append(data_entry)
        self.save_data()

    def run(self):
        try:
            rospy.loginfo("Starting data collection...")
            for n in range(self.iterations):
                # Spawn object
                spawn_msg = self.spawn_object()

                if spawn_msg is None:
                    continue

                object_id = spawn_msg.object_id

                for i in range(len(self.robot_poses)):
                    rospy.loginfo(f"Starting data collection cycle {i + 1}")

                    # Move robot
                    target_pose = self.robot_poses[i]
                    self.move_robot(target_pose)

                    # Capture image
                    image_name = f"image_{n}_pose{i + 1}"
                    self.take_picture(image_name)

                    # Save data
                    self.save_transformation_data(image_name, object_id, spawn_msg, self.robot_poses[i])

                # Delete the spawned object
                self.delete_object(object_id)

            rospy.loginfo("Data collection complete.")
        except rospy.ROSInterruptException:
            rospy.loginfo("Interrupted before completion.")


if __name__ == "__main__":
    collector = DataCollector()
    collector.run()
