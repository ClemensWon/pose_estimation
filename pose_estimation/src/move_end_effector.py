#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_gripper(position):
    """
    Sends a command to the gripper controller to move the gripper to a specific position.
    :param position: Desired position of the gripper (float).
    """
    rospy.loginfo(f"Commanding gripper to position={position}")

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = ['robotiq_85_left_knuckle_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [position]  # e.g. 0.0 -> closed, 0.04 -> open
    point.velocities = [0.0]
    point.time_from_start = rospy.Duration(1.0)
    trajectory_msg.points = [point]

    gripper_pub.publish(trajectory_msg)
    rospy.loginfo(f"Gripper command sent: {position}")


def move_end_effector(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, grip_pos=None):
    """
    Moves the robot's end effector to the specified position (x, y, z) and orientation (roll, pitch, yaw).
    If grip_pos is given (float), then the gripper is moved after the arm motion finishes.
    """
    try:
        rospy.loginfo(
            f"Moving end effector to: pos=({x}, {y}, {z}), rpy=({roll}, {pitch}, {yaw})"
        )

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

        # If a gripper position is provided, move the gripper after the arm motion
        if grip_pos is not None:
            move_gripper(grip_pos)

    except Exception as e:
        rospy.logerr(f"Error moving end-effector: {e}")
        status_publisher.publish("Error")


def main():
    global move_group, status_publisher, gripper_pub

    # Initialize MoveIt Commander and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_end_effector", anonymous=True)

    # Create the MoveGroupCommander for the manipulator
    move_group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=30)
    
    # Publisher for movement status
    status_publisher = rospy.Publisher("/movement_status", String, queue_size=10)

    # Publisher for gripper commands
    gripper_pub = rospy.Publisher("/ur5/gripper_controller/command", JointTrajectory, queue_size=10)

    # --- Command-line usage part ---
    # If user provided at least x, y, z
    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        roll  = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        yaw   = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
        # If the user provides a 7th argument for the gripper
        grip_pos = float(sys.argv[7]) if len(sys.argv) > 7 else None

        move_end_effector(x, y, z, roll, pitch, yaw, grip_pos)
    else:
        rospy.logerr("Usage: rosrun pose_estimation move_end_effector.py <x> <y> <z> [roll] [pitch] [yaw] [gripper_pos]")
        rospy.logerr("Example: rosrun pose_estimation move_end_effector.py 0.4 0.2 0.3 0.0 0.0 0.0 0.04")
        sys.exit(1)

    # Once done, we can exit immediately or keep spinning if desired.
    # If you prefer to see logs or wait, uncomment the following line:
    # rospy.spin()

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()