#!/usr/bin/env python3

import os
import rospy
import random
import json
import rospkg
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty, String
from pose_estimation.msg import ObjectSpawned, DeleteObject
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from moveit_commander import MoveGroupCommander

class DataCollector:
    def __init__(self):
        rospy.init_node("data_collector")

        # Initialize publishers and subscribers
        self.spawn_trigger_pub = rospy.Publisher("spawn_trigger", Empty, queue_size=10)
        self.object_spawned_sub = rospy.Subscriber("object_spawned", ObjectSpawned, queue_size=10)
        self.delete_object_pub = rospy.Publisher("delete_object", DeleteObject, queue_size=10)
        self.take_picture_pub = rospy.Publisher("/take_picture", String, queue_size=10)

        # Initialize MoveGroup for robot control
        self.move_group = MoveGroupCommander("manipulator")

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

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

        self.robot_poses = [
            [-0.2, -1.57, 0.0, 0.0, 0.0, -0.735],
            [-1.67, 0.0, 0.0, -3.14, 2.2, 0.0],
            [1.57, 0.0, 0.0, -3.14, -2.2, 0.0]
        ]

    def move_robot(self, target_pose):
        rospy.loginfo(f"Moving robot to pose: {target_pose}")
        self.move_group.set_joint_value_target(target_pose)
        self.move_group.go(wait=True)

    def spawn_object(self):
        rospy.loginfo("Spawning object...")
        self.spawn_trigger_pub.publish(Empty())

        try:
            spawn_msg = rospy.wait_for_message("/object_spawned", ObjectSpawned, timeout=10)
            rospy.loginfo(
                f"Object spawned: ID={spawn_msg.object_id}, Type={spawn_msg.object_type}, Position=({spawn_msg.pose.position.x}, {spawn_msg.pose.position.y}, {spawn_msg.pose.position.z})")
            return spawn_msg
        except rospy.ROSException:
            rospy.logerr("Timed out waiting for object to spawn.")
            return None

    def delete_object(self, object_id):
        rospy.loginfo(f"Deleting object with ID: {object_id}")
        delete_msg = DeleteObject()
        delete_msg.object_id = object_id
        self.delete_object_pub.publish(delete_msg)

    def take_picture(self, image_name):
        rospy.loginfo(f"Capturing image: {image_name}")
        self.current_image_name = image_name
        self.take_picture_pub.publish(image_name)
        rospy.sleep(2)

    def save_data(self):
        dataset_file = os.path.join(self.dataset_folder, "dataset.json")
        with open(dataset_file, "w") as f:
            json.dump(self.dataset, f, indent=4)
        rospy.loginfo(f"Dataset saved to {dataset_file}")

    def run(self):
        try:
            rospy.loginfo("Starting data collection...")
            for n in range(1):
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
                        }
                    }
                    self.dataset.append(data_entry)

                # Delete the spawned object
                self.delete_object(object_id)

            # Save all collected data to JSON
            self.save_data()
            rospy.loginfo("Data collection complete.")
        except rospy.ROSInterruptException:
            rospy.loginfo("Interrupted before completion.")

if __name__ == "__main__":
    collector = DataCollector()
    collector.run()
