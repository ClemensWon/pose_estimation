#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import rospkg


class TriggeredImageSaver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('triggered_image_saver', anonymous=True)

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Set the image topic
        self.image_topic = rospy.get_param('~image_topic', '/image_raw')

        # Get the path to the package folder
        rospack = rospkg.RosPack()
        package_name = rospy.get_param('~package_name', 'pose_estimation')
        package_path = rospack.get_path(package_name)

        # Set the folder for saving images
        self.save_folder = os.path.join(package_path, 'saved_images')
        os.makedirs(self.save_folder, exist_ok=True)

        rospy.loginfo(f"Images will be saved in: {self.save_folder}")

        # Store the latest image
        self.latest_image = None

        # Persistent subscriber to the image topic
        self.image_subscriber = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        # Subscribe to the trigger topic
        rospy.Subscriber('/take_picture', String, self.trigger_callback)

        rospy.loginfo("Node is ready. Waiting for image name messages on /take_picture.")

    def image_callback(self, msg):
        # Cache the latest image
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to process image: {str(e)}")

    def trigger_callback(self, msg):
        image_name = msg.data
        rospy.loginfo(f"Signal received to capture image with name: {image_name}")

        # Check if an image is available
        if self.latest_image is None:
            rospy.logerr("No image received yet!")
            return

        # Save the cached image
        filename = f"{image_name}.jpg"
        file_path = os.path.join(self.save_folder, filename)
        cv2.imwrite(file_path, self.latest_image)

        rospy.loginfo(f"Image saved successfully as {file_path}")


if __name__ == '__main__':
    try:
        TriggeredImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
