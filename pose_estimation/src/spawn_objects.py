#!/usr/bin/env python3

import rospy
import random
import os
import uuid
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Empty
from pose_estimation.msg import ObjectSpawned, DeleteObject


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

        # publishers
        self.object_spawned_pub = rospy.Publisher('object_spawned', ObjectSpawned, queue_size=10)

        # subscribers
        rospy.Subscriber("spawn_trigger", Empty, self.spawn_trigger_callback)
        rospy.Subscriber("delete_object", DeleteObject, self.delete_object_callback)

    def get_object_urdf(self, object_type):
        package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        urdf_path = os.path.join(package_path, 'urdf', f'{object_type}.urdf')
        
        try:
            with open(urdf_path, 'r') as file:
                return file.read()
        except Exception as e:
            rospy.logerr(f"Failed to load URDF file for {object_type}: {e}")
            return None

    def spawn_object(self, pose, object_type=None):
        if object_type is None:
            object_type = random.choice(self.enabled_objects)
        
        object_urdf = self.get_object_urdf(object_type)
        if object_urdf is None:
            return False

        # Generate a unique identifier
        object_id = str(uuid.uuid4())
        object_name = f"{object_type}_{object_id}"

        try:
            self.spawn_model(object_name, object_urdf, "", pose, "world")
            
            self.spawned_objects[object_id] = object_name

            spawn_msg = ObjectSpawned()
            spawn_msg.object_id = object_id
            spawn_msg.object_type = object_type
            spawn_msg.pose = pose
            self.object_spawned_pub.publish(spawn_msg)

            rospy.loginfo(f"Spawned {object_name} with ID {object_id} at position "
                        f"x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service call failed: {e}")
            return False

    def spawn_trigger_callback(self, msg):
        """Callback for simple trigger - spawns object at random position"""
        pose = Pose()
        pose.position.x = self.center_x + random.uniform(-self.x_length, self.x_length)
        pose.position.y = self.center_y + random.uniform(-self.y_length, self.y_length)
        pose.position.z = self.center_z

        x_orientation = self.x_degree + random.uniform(-self.x_degree_range, self.x_degree_range)
        y_orientation = self.y_degree + random.uniform(-self.y_degree_range, self.y_degree_range)
        z_orientation = self.z_degree + random.uniform(-self.z_degree_range, self.z_degree_range)
        pose.orientation = Quaternion(x_orientation, y_orientation, z_orientation, 1)

        self.spawn_object(pose)

    def delete_object_callback(self, msg):
        """Handle object deletion requests"""
        if msg.object_id in self.spawned_objects:  # Changed from msg.guid
            try:
                object_name = self.spawned_objects[msg.object_id]  # Changed from msg.object_id
                self.delete_model(object_name)
                del self.spawned_objects[msg.object_id]  # Changed from msg.guid
                rospy.loginfo(f"Deleted object with ID {msg.object_id}")  # Changed from msg.guid
            except rospy.ServiceException as e:
                rospy.logerr(f"Delete service call failed for ID {msg.object_id}: {e}")  # Changed from msg.guid
        else:
            rospy.logwarn(f"No object found with ID {msg.object_id}")  # Changed from msg.guid


if __name__ == '__main__':
    try:
        spawner = ObjectSpawner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
