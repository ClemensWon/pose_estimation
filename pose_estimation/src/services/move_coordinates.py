#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
from pose_estimation.srv import MoveToPose, MoveToPoseResponse

class EndEffectorMover:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("end_effector_mover")

        # Initialize MoveIt Commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        self.move_group.set_planning_time(10)

        # Advertise the service
        self.service = rospy.Service('move_to_position', MoveToPose, self.handle_move_to_position)
        rospy.loginfo("Service 'move_to_position' ready.")

    def handle_move_to_position(self, req):
        # Construct the target pose
        target_pose = Pose()
        target_pose.position.x = req.translation[0]
        target_pose.position.y = req.translation[1]
        target_pose.position.z = req.translation[2]
        
        # Set a default orientation (e.g., pointing down)
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0

        # Move the robot
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.go(wait=True)

        # Stop and clear targets
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Return service response
        response = MoveToPoseResponse()
        if success:
            response.success = True
            response.message = "Move successful"
            rospy.loginfo(response.message)
        else:
            response.success = False
            response.message = "Move failed"
            rospy.logerr(response.message)
        return response

if __name__ == "__main__":
    try:
        EndEffectorMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
