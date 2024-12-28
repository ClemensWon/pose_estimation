#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import rospkg
from pose_estimation.srv import SaveImage, SaveImageResponse

class ImageSaverService:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_saver_service', anonymous=True)

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
        self.image_subscriber = rospy.Subscriber(
            self.image_topic, 
            Image, 
            self.image_callback
        )

        # Create the service
        self.service = rospy.Service(
            'save_image', 
            SaveImage, 
            self.handle_save_image
        )

        rospy.loginfo("Image saver service is ready.")

    def image_callback(self, msg):
        # Cache the latest image
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to process image: {str(e)}")

    def handle_save_image(self, req):
        """Service handler for saving images."""
        response = SaveImageResponse()
        
        try:
            # Check if an image is available
            if self.latest_image is None:
                response.success = False
                response.message = "No image received yet!"
                response.file_path = ""
                return response

            # Save the cached image
            filename = f"{req.image_name}.jpg"
            file_path = os.path.join(self.save_folder, filename)
            cv2.imwrite(file_path, self.latest_image)

            response.success = True
            response.message = "Image saved successfully"
            response.file_path = file_path
            rospy.loginfo(f"Image saved successfully as {file_path}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to save image: {str(e)}"
            response.file_path = ""
            rospy.logerr(response.message)

        return response

def main():
    image_saver = ImageSaverService()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
