#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_gripper(gripper_pub, position):
    """
    Opens/closes the gripper by publishing a JointTrajectory.
      position ~ 0.0 (closed) to about 0.04 (open)
    """
    rospy.loginfo(f"Gripper -> position = {position}")

    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['robotiq_85_left_knuckle_joint']

    point = JointTrajectoryPoint()
    point.positions = [position]
    point.time_from_start = rospy.Duration(1.0)

    traj_msg.points = [point]
    gripper_pub.publish(traj_msg)

def main():
    if len(sys.argv) < 8:
        rospy.logerr("Usage: rosrun pose_estimation move_ik.py x y z qx qy qz qw")
        sys.exit(1)

    # Parse command-line arguments
    x  = float(sys.argv[1])
    y  = float(sys.argv[2])
    z  = float(sys.argv[3])
    qx = float(sys.argv[4])
    qy = float(sys.argv[5])
    qz = float(sys.argv[6])
    qw = float(sys.argv[7])

    # Initialize ROS and MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_ik_node", anonymous=True)

    # Create MoveGroup for the UR5 manipulator
    move_group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=30)

    # Publisher for the gripper
    gripper_pub = rospy.Publisher("/ur5/gripper_controller/command", JointTrajectory, queue_size=10)

    rospy.loginfo("Waiting a moment for the gripper topic to be ready...")
    rospy.sleep(1.0)  # let publisher connect

    # 1) Open the gripper (example: 0.04 rad or meters for open)
    move_gripper(gripper_pub, position=0.04)
    rospy.sleep(1.0)

    # 2) Set the target pose
    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.x = qx
    target_pose.orientation.y = qy
    target_pose.orientation.z = qz
    target_pose.orientation.w = qw

    move_group.set_pose_target(target_pose)

    # 3) Plan and move
    plan_success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if plan_success:
        rospy.loginfo("Successfully moved to the target pose.")
    else:
        rospy.logerr("Failed to move to the target pose.")
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("Pose target unreachable.")
        sys.exit(1)

    # 4) Close the gripper (0.0 -> closed)
    move_gripper(gripper_pub, position=0.0)
    rospy.sleep(1.0)

    rospy.loginfo("Done. Shutting down MoveIt.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
