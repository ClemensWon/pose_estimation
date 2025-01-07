#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from pose_estimation.srv import MoveSingleJoint, MoveSingleJointResponse

class SingleJointMover:
    def __init__(self):
        # Initialize MoveIt and the ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_single_joint", anonymous=True)

        # Create a MoveGroupCommander for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=30)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.move_group.set_planning_time(4)  
        self.move_group.set_num_planning_attempts(10)

        # Create the service
        self.service = rospy.Service('move_single_joint', MoveSingleJoint, self.handle_move_single_joint)

        rospy.loginfo("Single Joint Mover service is ready.")

    def move_single_joint(self, joint_index, joint_value):
        """
        Move a single joint to a specified value.
        """
        joint_values = self.move_group.get_current_joint_values()
        rospy.loginfo(f"Current Joint Values: {joint_values}")

        # Check if joint index is valid
        if joint_index < 0 or joint_index >= len(joint_values):
            rospy.logerr(f"Invalid joint index: {joint_index}. Must be in range [0, {len(joint_values)-1}].")
            return False, f"Invalid joint index: {joint_index}"

        # Check joint limits (optional)
        rospy.loginfo(f"Setting joint {joint_index} to {joint_value:.4f} radians.")
        
        # Set the specific joint value
        joint_values[joint_index] = joint_value
        self.move_group.set_joint_value_target(joint_values)

        # Plan and execute
        success = self.move_group.go(wait=True)
        if success:
            rospy.loginfo(f"Successfully moved joint {joint_index} to {joint_value:.4f}.")
            return True, f"Successfully moved joint {joint_index} to {joint_value:.4f}"
        else:
            rospy.logerr(f"Failed to move joint {joint_index} to {joint_value:.4f}.")
            return False, f"Failed to move joint {joint_index} to {joint_value:.4f}"


    def handle_move_single_joint(self, req):
        """
        Service handler for moving a single joint.
        """
        rospy.loginfo(f"Received request to move joint {req.joint_index} to value {req.joint_value}")
        success, message = self.move_single_joint(req.joint_index, req.joint_value)
        return MoveSingleJointResponse(success=success, message=message)

    def shutdown(self):
        """
        Clean shutdown of the node
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Single Joint Mover node shut down.")

def main():
    single_joint_mover = SingleJointMover()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        single_joint_mover.shutdown()

if __name__ == "__main__":
    main()
