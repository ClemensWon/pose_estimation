#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from pose_estimation.srv import MoveJoints, MoveJointsResponse

class JointMover:
    def __init__(self):
        # Initialize MoveIt and the ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("joint_mover", anonymous=True)

        # Create a MoveGroupCommander for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=30)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.move_group.set_planning_time(0.5)


        # Create the service
        self.service = rospy.Service('move_joints', MoveJoints, self.handle_move_joints)

        rospy.loginfo("Joint mover service is ready.")

    def handle_move_joints(self, req):
        """
        Service handler for moving the robot to specified joint values.
        """
        response = MoveJointsResponse()

        try:
            rospy.loginfo("Received joint values: %s", req.joint_values)

            # Set the target joint values
            self.move_group.set_joint_value_target(req.joint_values)

            # Plan and execute the motion
            success = self.move_group.go(wait=True)
            
            if success:
                response.success = True
                response.message = "Robot moved successfully to the specified joint values!"
                rospy.loginfo(response.message)
            else:
                response.success = False
                response.message = "Failed to move the robot to the specified joint values."
                rospy.logerr(response.message)

            # Stop the robot and clear targets
            self.move_group.stop()
            self.move_group.clear_pose_targets()

        except Exception as e:
            response.success = False
            response.message = f"An error occurred: {str(e)}"
            rospy.logerr(response.message)

        return response

    def shutdown(self):
        """
        Clean shutdown of the node
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Joint mover node shut down.")

def main():
    joint_mover = JointMover()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        joint_mover.shutdown()

if __name__ == "__main__":
    main()
