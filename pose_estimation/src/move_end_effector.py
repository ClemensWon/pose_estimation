#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def move_end_effector(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """
    Moves the robot's end effector to the specified position (x, y, z) and orientation (roll, pitch, yaw).
    """
    global move_group
    
    try:
        rospy.loginfo("Moving end effector to: "
                      f"pos=({x}, {y}, {z}), rpy=({roll}, {pitch}, {yaw})")

        # Define the target pose
        pose_target = Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z

        # Convert roll, pitch, yaw to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]

        # Send goal
        move_group.set_pose_target(pose_target)
        success = move_group.go(wait=True)

        if success:
            rospy.loginfo("End-effector successfully moved to the target pose.")
            status_publisher.publish("Success")
        else:
            rospy.logerr("Failed to move end-effector to the target pose.")
            status_publisher.publish("Failure")

        # Stop any residual movement and clear targets
        move_group.stop()
        move_group.clear_pose_targets()

    except Exception as e:
        rospy.logerr(f"Error moving end-effector: {e}")
        status_publisher.publish("Error")


def move_end_effector_callback(msg):
    """
    Subscriber callback: Interprets Float64MultiArray as [x, y, z, (roll), (pitch), (yaw)]
    and calls move_end_effector.
    """
    data = msg.data
    num_vals = len(data)
    if num_vals < 3:
        rospy.logerr("Received less than 3 elements. Need at least x, y, z.")
        return

    x = data[0]
    y = data[1]
    z = data[2]
    roll  = data[3] if num_vals >= 4 else 0.0
    pitch = data[4] if num_vals >= 5 else 0.0
    yaw   = data[5] if num_vals >= 6 else 0.0

    rospy.loginfo(f"Subscriber callback received: {data}")
    move_end_effector(x, y, z, roll, pitch, yaw)


def main():
    global move_group, status_publisher

    # Initialize MoveIt commander and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_end_effector", anonymous=True)

    # Create the MoveGroupCommander for the manipulator
    move_group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=30)
    
    # Publisher for movement status
    status_publisher = rospy.Publisher("/movement_status", String, queue_size=10)

    # Subscriber for end-effector move commands (in Float64MultiArray form)
    rospy.Subscriber("/joint_values", Float64MultiArray, move_end_effector_callback)

    rospy.loginfo("move_end_effector node is up and listening to /joint_values. "
                  "Also accepts command-line arguments for one-time motion.")

    # --- Command-line usage part ---
    # If user provided at least x, y, z
    if len(sys.argv) >= 4:
        # Parse mandatory arguments
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

        # Parse optional orientation arguments
        roll  = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        yaw   = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

        # Move once immediately
        move_end_effector(x, y, z, roll, pitch, yaw)
    else:
        rospy.logwarn("No command-line position arguments given. "
                      "Node will only respond to /joint_values subscriber.")

    # Keep node alive for subscriber callbacks
    rospy.spin()

    # Clean shutdown
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
