#!/usr/bin/env python3

import sys
import rospy
import tf
import tf.transformations as tft
import moveit_commander
from geometry_msgs.msg import Pose
from pose_estimation.srv import MoveToPose, MoveToPoseResponse


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

        # Initialize TF listener
        self.tf_listener = tf.TransformListener()

        # Create the service
        self.service = rospy.Service('move_to_pose', MoveToPose, self.handle_move_to_pose)
        rospy.loginfo("Pose mover service is ready.")

    def get_world_to_tool0_pose(self):
        """
        Look up the pose of 'tool0' in the 'world' frame via TF.
        Returns a geometry_msgs/Pose or None if lookup fails.
        """
        try:
            # Wait for up to 2 seconds for transform
            self.tf_listener.waitForTransform("world", "tool0", rospy.Time(0), rospy.Duration(2.0))
            (trans, rot) = self.tf_listener.lookupTransform("world", "tool0", rospy.Time(0))
            
            pose = Pose()
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Could not find transform world->tool0: {e}")
            return None

    def handle_move_to_pose(self, req):
        """
        Service handler for moving the robot so that tool0 aligns 
        with the object, given T_tool0_object (translation, rotation).
        """
        response = MoveToPoseResponse()

        try:
            rospy.loginfo("Received request to move tool0 to object pose in tool0 frame.")
            rospy.loginfo(f"Translation: {req.translation}, Rotation: {req.rotation}")
            rospy.loginfo(f"Image Name: {req.image_name}, Object ID: {req.object_id}, Object Type: {req.object_type}")

            # 1) Retrieve current T_world_tool0 from TF
            world_tool0_pose = self.get_world_to_tool0_pose()
            if world_tool0_pose is None:
                response.success = False
                response.message = "Failed to lookup T_world_tool0 from TF."
                rospy.logerr(response.message)
                return response

            # 2) Construct Pose for T_tool0_object
            tool0_object_pose = Pose()
            tool0_object_pose.position.x = req.translation[0]
            tool0_object_pose.position.y = req.translation[1]
            tool0_object_pose.position.z = req.translation[2]
            tool0_object_pose.orientation.x = req.rotation[0]
            tool0_object_pose.orientation.y = req.rotation[1]
            tool0_object_pose.orientation.z = req.rotation[2]
            tool0_object_pose.orientation.w = req.rotation[3]

            # 3) Convert both poses to 4x4 matrices
            mat_world_tool0   = pose_to_matrix(world_tool0_pose)
            mat_tool0_object  = pose_to_matrix(tool0_object_pose)

            # 4) Compute T_world_object = T_world_tool0 * T_tool0_object
            mat_world_object = tft.concatenate_matrices(mat_world_tool0, mat_tool0_object)
            world_object_pose = matrix_to_pose(mat_world_object)

            # 5) Now set this as the MoveIt target
            self.move_group.set_pose_target(world_object_pose)

            # Plan and execute the motion
            success = self.move_group.go(wait=True)

            if success:
                response.success = True
                response.message = "Robot moved successfully to the object pose (tool0->object)!"
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
