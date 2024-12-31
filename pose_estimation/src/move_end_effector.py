#!/usr/bin/env python3

import rospy
import sys
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def move_end_effector(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    # Initialize MoveIt! Commander
    rospy.init_node('move_end_effector', anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("manipulator")  # Replace "manipulator" with your MoveIt! group name

    # Define target pose
    pose_target = Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    # Convert roll, pitch, yaw to a quaternion
    q = quaternion_from_euler(roll, pitch, yaw)
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]

    # Set the target pose
    group.set_pose_target(pose_target)

    # Plan and execute the motion
    plan = group.go(wait=True)

    # Stop the robot and clear targets
    group.stop()
    group.clear_pose_targets()

    rospy.loginfo(f"End-effector moved to target position: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")

if __name__ == "__main__":
    try:
        # Check if enough arguments are provided
        if len(sys.argv) < 4:
            rospy.logerr("Usage: rosrun your_package_name move_end_effector.py <x> <y> <z> [roll] [pitch] [yaw]")
            rospy.logerr("Example: rosrun your_package_name move_end_effector.py 0.4 0.2 0.3 0.0 0.0 0.0")
            sys.exit(1)

        # Parse mandatory arguments
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

        # Parse optional arguments for roll, pitch, yaw
        roll = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

        # Move the end-effector to the specified coordinates
        move_end_effector(x, y, z, roll, pitch, yaw)
    except rospy.ROSInterruptException:
        pass
