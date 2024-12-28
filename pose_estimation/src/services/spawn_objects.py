#!/usr/bin/env python3

import rospy
import rospkg

import random
import os
import uuid
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion
from pose_estimation.srv import SpawnObject, SpawnObjectResponse
from pose_estimation.srv import DeleteObject, DeleteObjectResponse


class ObjectSpawner:

    def __init__(self):
        rospy.init_node('spawn_objects_node')

        self.spawned_objects = {}

        # read config
        self.config = rospy.get_param('/spawn_objects')

        self.center_x = self.config['spawn_center']['pos_x']
        self.center_y = self.config['spawn_center']['pos_y']
        self.center_z = self.config['spawn_center']['pos_z']
        self.x_length = self.config['spawn_radius']['x_length']
        self.y_length = self.config['spawn_radius']['y_length']
        
        self.x_degree = self.config['orientation']['x_degree']
        self.y_degree = self.config['orientation']['y_degree']
        self.z_degree = self.config['orientation']['z_degree']

        self.x_degree_range = self.config['orientation_range']['x_degree']
        self.y_degree_range = self.config['orientation_range']['y_degree']
        self.z_degree_range = self.config['orientation_range']['z_degree']
        
        self.enabled_objects = []
        if self.config['object_types']['cube']:
            self.enabled_objects.append('cube')
        if self.config['object_types']['cylinders']:
            self.enabled_objects.append('cylinder')
        if self.config['object_types']['spheres']:
            self.enabled_objects.append('sphere')
        
        if not self.enabled_objects:
            rospy.logerr("No object types enabled in configuration!")
            return

        rospy.wait_for_service('gazebo/spawn_urdf_model')
        rospy.wait_for_service('gazebo/delete_model')
        self.spawn_model = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

        # Services
        self.spawn_service = rospy.Service('spawn_object', SpawnObject, self.handle_spawn_request)
        self.delete_service = rospy.Service('delete_object', DeleteObject, self.handle_delete_request)

    def get_object_urdf(self, object_type):
        package_path = rospkg.RosPack().get_path('pose_estimation')
        urdf_path = os.path.join(package_path, 'urdf', f'{object_type}.urdf')
        
        try:
            with open(urdf_path, 'r') as file:
                return file.read()
        except Exception as e:
            rospy.logerr(f"Failed to load URDF file for {object_type}: {e}")
            return None

    def spawn_object(self):
        """Generate random pose and spawn object"""
        pose = Pose()
        pose.position.x = self.center_x + random.uniform(-self.x_length, self.x_length)
        pose.position.y = self.center_y + random.uniform(-self.y_length, self.y_length)
        pose.position.z = self.center_z

        x_orientation = self.x_degree + random.uniform(0, self.x_degree_range)
        y_orientation = self.y_degree + random.uniform(0, self.y_degree_range)
        z_orientation = self.z_degree + random.uniform(0, self.z_degree_range)

        
            # Convert Euler angles to quaternion
        q = quaternion_from_euler(x_orientation, y_orientation, z_orientation)

        rospy.loginfo(f"x: {q[0]}, y: {q[1]}, z: {q[2]}, w: {q[3]}")
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        object_type = random.choice(self.enabled_objects)
        object_urdf = self.get_object_urdf(object_type)
        if object_urdf is None:
            return None, None, None, False, "Failed to load URDF"

        # Generate a unique identifier
        object_id = str(uuid.uuid4())
        object_name = f"{object_type}_{object_id}"

        try:
            self.spawn_model(object_name, object_urdf, "", pose, "world")
            self.spawned_objects[object_id] = object_name

            rospy.loginfo(f"Spawned {object_name} with ID {object_id} at position "
                         f"x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
            return object_id, object_type, pose, True, "Object spawned successfully"
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service call failed: {e}")
            return None, None, None, False, str(e)

    def handle_spawn_request(self, req):
        """Service handler for spawn requests"""
        response = SpawnObjectResponse()
        object_id, object_type, pose, success, message = self.spawn_object()
        
        if success:
            response.object_id = object_id
            response.object_type = object_type
            response.pose = pose
            response.success = True
            response.message = message
        else:
            response.success = False
            response.message = message
        
        return response

    def handle_delete_request(self, req):
        """Service handler for delete requests"""
        response = DeleteObjectResponse()
        
        if req.object_id in self.spawned_objects:
            try:
                object_name = self.spawned_objects[req.object_id]
                self.delete_model(object_name)
                del self.spawned_objects[req.object_id]
                response.success = True
                response.message = f"Successfully deleted object with ID {req.object_id}"
                rospy.loginfo(f"Deleted object with ID {req.object_id}")
            except rospy.ServiceException as e:
                response.success = False
                response.message = f"Delete service call failed: {e}"
                rospy.logerr(f"Delete service call failed for ID {req.object_id}: {e}")
        else:
            response.success = False
            response.message = f"No object found with ID {req.object_id}"
            rospy.logwarn(f"No object found with ID {req.object_id}")
        
        return response


if __name__ == '__main__':
    try:
        spawner = ObjectSpawner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
