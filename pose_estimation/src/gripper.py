#!/usr/bin/env python3

import rospy
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_gripper(position):
    """
    Sends a command to the gripper controller to move the gripper to a specific position.

    :param position: Desired position of the gripper (float).
    """
    pub = rospy.Publisher('/ur5/gripper_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('gripper_control', anonymous=True)

    # Wait for the publisher to connect
    rospy.sleep(1)

    # Create a JointTrajectory message
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = ['robotiq_85_left_knuckle_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [position]
    point.velocities = [0.0]
    point.time_from_start = rospy.Duration(1.0)

    trajectory_msg.points = [point]

    # Publish the message
    pub.publish(trajectory_msg)
    rospy.loginfo(f"Command sent to gripper: position={position}")

if __name__ == "__main__":
    try:
        if len(sys.argv) != 2:
            rospy.logerr("Usage: rosrun your_package_name gripper.py <position>")
            rospy.logerr("Example: rosrun your_package_name gripper.py 0.0 (to close)")
            rospy.logerr("Example: rosrun your_package_name gripper.py 0.04 (to open)")
            sys.exit(1)

        position = float(sys.argv[1])  # Get position from command line arguments
        move_gripper(position)
    except rospy.ROSInterruptException:
        pass
