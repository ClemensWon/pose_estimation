#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from pose_estimation.srv import MoveToCoordinate, MoveToCoordinateResponse

class CoordinateMover:
    def __init__(self):
        # Initialize MoveIt and the ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_to_coordinate", anonymous=True)  # Ensure the name matches client

        # Create a MoveGroupCommander for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=30)
        self.move_group.set_planning_time(10)  
        self.move_group.set_num_planning_attempts(20) 
        self.move_group.set_planning_scene_interface(None)

        # Optionally, set the reference frame if needed
        self.move_group.set_pose_reference_frame("base_link")

        # Create the service
        self.service = rospy.Service('move_to_coordinate', MoveToCoordinate, self.handle_move_to_coordinate) 

        rospy.loginfo("Pose mover service 'move_to_coordinate' is ready.")

    def handle_move_to_coordinate(self, req):
        """
        Service handler for moving the robot's end-effector to a specified pose.
        """
        response = MoveToCoordinateResponse()

        try:
            target_coordinate = req.target_coordinate
            rospy.loginfo("Received target pose: Position(%f, %f, %f) Orientation(%f, %f, %f, %f)",
                          target_coordinate.position.x, target_coordinate.position.y, target_coordinate.position.z,
                          target_coordinate.orientation.x, target_coordinate.orientation.y,
                          target_coordinate.orientation.z, target_coordinate.orientation.w)

            # Set the target pose
            self.move_group.set_pose_target(target_coordinate)

            # Plan the motion
            plan = self.move_group.plan()

            if not plan or not plan[0]:
                response.success = False
                response.message = "Planning failed. No valid plan found."
                rospy.logerr(response.message)
                self.move_group.clear_pose_targets()
                return response

            rospy.loginfo("Plan found. Executing the plan...")

            # Execute the plan
            execute_success = self.move_group.go(wait=True)

            if execute_success:
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                response.success = True
                response.message = "Robot moved successfully to the specified pose!"
                rospy.loginfo(response.message)
            else:
                response.success = False
                response.message = "Failed to execute the movement to the specified pose."
                rospy.logerr(response.message)

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
        rospy.loginfo("Pose mover node shut down.")

def main():
    coordinate_mover = CoordinateMover()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        coordinate_mover.shutdown()

if __name__ == "__main__":
    main()
