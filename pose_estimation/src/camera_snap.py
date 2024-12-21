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

        # Placeholder for the image subscriber
        self.image_subscriber = None

        # Subscribe to the trigger topic
        rospy.Subscriber('/take_picture', String, self.trigger_callback)

        rospy.loginfo("Node is ready. Waiting for image name messages on /take_picture.")

    def trigger_callback(self, msg):
        image_name = msg.data
        rospy.loginfo(f"Signal received to capture image with name: {image_name}")

        # Subscribe to the image topic
        self.image_subscriber = rospy.Subscriber(self.image_topic, Image, self.image_callback, callback_args=image_name)

    def image_callback(self, msg, image_name):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Save the image with the name from the message
            filename = f"{image_name}.jpg"
            file_path = os.path.join(self.save_folder, filename)
            cv2.imwrite(file_path, cv_image)

            rospy.loginfo(f"Image saved successfully as {file_path}")

            # Unsubscribe from the image topic after capturing the image
            if self.image_subscriber:
                self.image_subscriber.unregister()
                self.image_subscriber = None
                rospy.loginfo("Unsubscribed from the image topic.")
        except Exception as e:
            rospy.logerr(f"Failed to save image: {str(e)}")


if __name__ == '__main__':
    try:
        TriggeredImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
