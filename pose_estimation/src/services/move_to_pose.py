#!/usr/bin/env python3

import sys
import rospy
import tf
import tf.transformations as tft
import moveit_commander
from geometry_msgs.msg import Pose
from pose_estimation.srv import MoveToPose, MoveToPoseResponse


def pose_to_matrix(pose):
    """Convert geometry_msgs/Pose to a 4x4 transformation matrix."""
    trans = (pose.position.x, pose.position.y, pose.position.z)
    rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    mat_trans = tft.translation_matrix(trans)
    mat_rot = tft.quaternion_matrix(rot)
    return tft.concatenate_matrices(mat_trans, mat_rot)


def matrix_to_pose(mat):
    """Convert a 4x4 transformation matrix to geometry_msgs/Pose."""
    trans = tft.translation_from_matrix(mat)
    quat = tft.quaternion_from_matrix(mat)

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
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        self.move_group.set_planning_time(5)
        self.move_group.set_num_planning_attempts(20)

        # Initialize TF listener
        self.tf_listener = tf.TransformListener()

        # Create the service
        self.service = rospy.Service('move_to_pose', MoveToPose, self.handle_move_to_pose)
        rospy.loginfo("Pose mover service is ready.")

    def lookup_transform(self, target_frame, source_frame, timeout=2.0):
        """
        Look up the transformation between two frames using TF.
        Returns a geometry_msgs/Pose or None if lookup fails.
        """
        try:
            self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout))
            (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))

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
            rospy.logwarn(f"Could not find transform {target_frame} -> {source_frame}: {e}")
            return None

    def compute_target_pose(self, world_tool0_pose, req_translation, req_rotation, offset=[0, 0, 0.1]):
        """
        Compute the target pose in the world frame based on:
        - Current world -> tool0 pose
        - Desired tool0 -> object translation
        - Optional offset in the tool0 -> object frame
        Note: Rotation is ignored.
        """
        # Construct T_tool0_object with only translation
        tool0_object_pose = Pose()
        tool0_object_pose.position.x = req_translation[0]
        tool0_object_pose.position.y = req_translation[1]
        tool0_object_pose.position.z = req_translation[2]
        # Set orientation to identity since it's ignored
        tool0_object_pose.orientation.x = 0.0
        tool0_object_pose.orientation.y = 0.0
        tool0_object_pose.orientation.z = 0.0
        tool0_object_pose.orientation.w = 1.0

        # Convert poses to transformation matrices
        mat_world_tool0 = pose_to_matrix(world_tool0_pose)
        mat_tool0_object = pose_to_matrix(tool0_object_pose)
        offset_matrix = tft.translation_matrix(offset)

        # Compute final transformation
        mat_tool0_object_with_offset = tft.concatenate_matrices(mat_tool0_object, offset_matrix)
        mat_world_object = tft.concatenate_matrices(mat_world_tool0, mat_tool0_object_with_offset)

        # Convert back to Pose
        return matrix_to_pose(mat_world_object)

    def handle_move_to_pose(self, req):
        """
        Service handler to move the robot to a specified position in the tool0 frame,
        ignoring orientation and moving in a straight line.
        """
        response = MoveToPoseResponse()

        try:
            rospy.loginfo("Received request to move tool0 to object position in tool0 frame.")
            rospy.loginfo(f"Translation: {req.translation}")

            # 1) Retrieve current T_world_tool0 from TF
            world_tool0_pose = self.lookup_transform("world", "tool0")
            if world_tool0_pose is None:
                response.success = False
                response.message = "Failed to lookup T_world_tool0 from TF."
                rospy.logerr(response.message)
                return response

            # 2) Compute the target pose in the world frame with offset
            target_pose = self.compute_target_pose(
                world_tool0_pose, req.translation, req.rotation, offset=[0, 0, 0.1]
            )

            # 3) Retrieve current end effector orientation
            current_pose = self.move_group.get_current_pose().pose
            target_pose.orientation = current_pose.orientation  # Preserve current orientation

            rospy.loginfo(f"Computed Target Position: x={target_pose.position.x}, "
                          f"y={target_pose.position.y}, z={target_pose.position.z}")
            rospy.loginfo(f"Preserved Orientation: x={target_pose.orientation.x}, "
                          f"y={target_pose.orientation.y}, z={target_pose.orientation.z}, "
                          f"w={target_pose.orientation.w}")

            rospy.loginfo("Runnnnnnnn")

            # 4) Plan a Cartesian path to the target position
            waypoints = []
            waypoints.append(target_pose)

            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0,         # jump_threshold
                True         # avoid_collisions
            )

            if fraction < 1.0:
                rospy.logwarn(f"Only {fraction*100:.2f}% of the path was planned. Planning failed.")
                response.success = False
                response.message = "Failed to plan a complete Cartesian path to the target position."
                return response

            # 5) Execute the plan
            self.move_group.execute(plan, wait=True)

            # 6) Verify the movement
            final_pose = self.move_group.get_current_pose().pose
            position_error = (
                abs(final_pose.position.x - target_pose.position.x),
                abs(final_pose.position.y - target_pose.position.y),
                abs(final_pose.position.z - target_pose.position.z)
            )

            rospy.loginfo(f"Final Position: x={final_pose.position.x}, y={final_pose.position.y}, z={final_pose.position.z}")
            rospy.loginfo(f"Position Error: x={position_error[0]}, y={position_error[1]}, z={position_error[2]}")

            if all(err < 1e-3 for err in position_error):
                response.success = True
                response.message = "Robot moved successfully to the target position!"
                rospy.loginfo(response.message)
            else:
                response.success = False
                response.message = "Robot did not reach the target position accurately."
                rospy.logerr(response.message)

            # Clean up after planning
            self.move_group.stop()
            self.move_group.clear_pose_targets()

        except Exception as e:
            response.success = False
            response.message = f"An error occurred: {str(e)}"
            rospy.logerr(response.message)

        return response

    def shutdown(self):
        """
        Clean shutdown of the node.
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
