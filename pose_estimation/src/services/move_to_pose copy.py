#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from pose_estimation.srv import MoveToPose, MoveToPoseResponse
import tf.transformations as tft
from geometry_msgs.msg import Pose

def pose_to_matrix(pose):
    """
    Convert geometry_msgs/Pose to a 4x4 transformation matrix.
    """
    trans = (pose.position.x, pose.position.y, pose.position.z)
    rot   = (pose.orientation.x, pose.orientation.y, 
             pose.orientation.z, pose.orientation.w)

    mat_trans = tft.translation_matrix(trans)
    mat_rot   = tft.quaternion_matrix(rot)
    return tft.concatenate_matrices(mat_trans, mat_rot)

def matrix_to_pose(mat):
    """
    Convert a 4x4 transformation matrix to geometry_msgs/Pose.
    """
    trans = tft.translation_from_matrix(mat)
    quat  = tft.quaternion_from_matrix(mat)

    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


class PoseMover:
    def __init__(self):
        # Initialize MoveIt and the ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pose_mover", anonymous=True)

        # Create a MoveGroupCommander for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=30)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.move_group.set_planning_time(4)
        self.move_group.set_num_planning_attempts(10)

        # Create the service
        self.service = rospy.Service('move_to_pose', MoveToPose, self.handle_move_to_pose)

        rospy.loginfo("Pose mover service is ready.")

    def handle_move_to_pose(self, req):
        """
        Service handler for moving the robot to a specified pose.
        """
        response = MoveToPoseResponse()

        try:
            rospy.loginfo("Received request to move to pose:")
            rospy.loginfo(f"Translation: {req.translation}, Rotation: {req.rotation}")
            rospy.loginfo(f"Image Name: {req.image_name}, Object ID: {req.object_id}, Object Type: {req.object_type}")

            # Construct the target pose
            target_pose = Pose()
            target_pose.position.x = req.translation[0]
            target_pose.position.y = req.translation[1]
            target_pose.position.z = req.translation[2]
            target_pose.orientation.x = req.rotation[0]
            target_pose.orientation.y = req.rotation[1]
            target_pose.orientation.z = req.rotation[2]
            target_pose.orientation.w = req.rotation[3]

            # Set the target pose
            self.move_group.set_pose_target(target_pose)

            # Plan and execute the motion
            success = self.move_group.go(wait=True)

            if success:
                response.success = True
                response.message = "Robot moved successfully to the specified pose!"
                rospy.loginfo(response.message)
            else:
                response.success = False
                response.message = "Failed to move the robot to the specified pose."
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
        rospy.loginfo("Pose mover node shut down.")

def main():
    pose_mover = PoseMover()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        pose_mover.shutdown()

if __name__ == "__main__":
    main()
