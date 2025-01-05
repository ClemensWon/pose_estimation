#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pose_estimation.srv import GripperControl, GripperControlResponse  # Define your service

class GripperControlService:
    def __init__(self):
        rospy.init_node("gripper_control_service", anonymous=True)

        # Publisher to the gripper controller topic
        self.gripper_pub = rospy.Publisher('/ur5/gripper_controller/command', JointTrajectory, queue_size=10)

        # Initialize the ROS service
        self.service = rospy.Service('gripper_control', GripperControl, self.handle_gripper_control)

        rospy.loginfo("Gripper control service is ready.")

    def handle_gripper_control(self, req):
        try:
            position = req.position  # Desired position for the gripper

            # Create and publish a JointTrajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['robotiq_85_left_knuckle_joint']
            
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.velocities = [0.0]
            point.time_from_start = rospy.Duration(1.0)

            trajectory_msg.points = [point]

            # Publish the command
            self.gripper_pub.publish(trajectory_msg)
            rospy.loginfo(f"Gripper command sent: position={position}")

            # Allow time for the gripper to move
            rospy.sleep(1.0)  # Adjust the duration if necessary

            return GripperControlResponse(success=True, message="Gripper moved successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to move gripper: {str(e)}")
            return GripperControlResponse(success=False, message=f"Error: {str(e)}")


def main():
    gripper_service = GripperControlService()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gripper control service interrupted.")

if __name__ == "__main__":
    main()
