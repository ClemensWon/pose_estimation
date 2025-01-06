#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from pose_estimation.srv import MoveToCoordinate, MoveToCoordinateResponse
from visualization_msgs.msg import Marker

class CoordinateMover:
    def __init__(self):
        # Initialize MoveIt and the ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_to_coordinate", anonymous=True)  # Ensure the name matches client
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        # Create a MoveGroupCommander for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=30)
        self.move_group.set_planning_time(10)  
        self.move_group.set_num_planning_attempts(20) 


        # Optionally, set the reference frame if needed
        self.move_group.set_pose_reference_frame("world")

        # Create the service
        self.service = rospy.Service('move_to_coordinate', MoveToCoordinate, self.handle_move_to_coordinate) 

        rospy.loginfo("Pose mover service 'move_to_coordinate' is ready.")
        
    def publish_marker(self, target_pose):
        """
        Publishes a marker in RViz to visualize the target pose.
        Args:
            target_pose (Pose): The target pose to visualize.
        """
        marker = Marker()
        marker.header.frame_id = "world"  # Match the reference frame of the pose
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_pose"
        marker.id = 0
        marker.type = Marker.ARROW  # You can change to SPHERE, CUBE, etc.
        marker.action = Marker.ADD

        # Set marker position
        marker.pose.position = target_pose.position
        marker.pose.orientation = target_pose.orientation

        # Set marker scale (adjust as needed)
        marker.scale.x = 0.1  # Length of the arrow
        marker.scale.y = 0.02  # Width of the arrow
        marker.scale.z = 0.02  # Height of the arrow

        # Set marker color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Publish the marker
        self.marker_pub.publish(marker)
        rospy.loginfo("Published marker for target pose.")

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
            
            self.publish_marker(target_coordinate)
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
