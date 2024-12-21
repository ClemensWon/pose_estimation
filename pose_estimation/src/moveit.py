#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Float64MultiArray, String
import moveit_commander

def joint_values_callback(data):
    """
    Callback function for the subscriber. Receives joint values and moves the robot.
    """
    global move_group

    rospy.loginfo("Received joint values: %s", data.data)

    # Set the target joint values
    joint_positions = data.data
    move_group.set_joint_value_target(joint_positions)

    # Plan and execute the motion
    success = move_group.go(wait=True)
    if success:
        rospy.loginfo("Robot moved successfully to the specified joint values!")
        status_publisher.publish("Success")
    else:
        rospy.logerr("Failed to move the robot to the specified joint values.")
        status_publisher.publish("Failure")

    # Stop the robot and clear targets
    move_group.stop()
    move_group.clear_pose_targets()

def main():
    global move_group, status_publisher

    # Initialize MoveIt and the ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("joint_mover", anonymous=True)

    # Create a MoveGroupCommander for the manipulator
    move_group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=30)

    # Initialize a publisher for status updates
    status_publisher = rospy.Publisher("/movement_status", String, queue_size=10)

    # Initialize a subscriber to listen for joint values
    rospy.Subscriber("/joint_values", Float64MultiArray, joint_values_callback)

    rospy.loginfo("Joint mover node is ready and waiting for joint values.")

    # Keep the node running
    rospy.spin()

    # Shut down MoveIt Commander
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
