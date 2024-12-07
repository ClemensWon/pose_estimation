#!/usr/bin/env python3

import rospy
import random
import yaml
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def get_object_urdf(object_type):
    if object_type == "cube":
        return """
        <?xml version="1.0"?>
        <robot name="cube">
            <link name="cube_link">
                <visual>
                    <geometry>
                        <box size="0.05 0.05 0.05"/>
                    </geometry>
                    <material name="blue">
                        <color rgba="0 0 1 1"/>
                    </material>
                </visual>
                <collision>
                    <geometry>
                        <box size="0.05 0.05 0.05"/>
                    </geometry>
                </collision>
                <inertial>
                    <mass value="0.1"/>
                    <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
                </inertial>
            </link>
        </robot>
        """
    elif object_type == "cylinder":
        return """
        <?xml version="1.0"?>
        <robot name="cylinder">
            <link name="cylinder_link">
                <visual>
                    <geometry>
                        <cylinder radius="0.025" length="0.05"/>
                    </geometry>
                    <material name="red">
                        <color rgba="1 0 0 1"/>
                    </material>
                </visual>
                <collision>
                    <geometry>
                        <cylinder radius="0.025" length="0.05"/>
                    </geometry>
                </collision>
                <inertial>
                    <mass value="0.1"/>
                    <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
                </inertial>
            </link>
        </robot>
        """
    elif object_type == "sphere":
        return """
        <?xml version="1.0"?>
        <robot name="sphere">
            <link name="sphere_link">
                <visual>
                    <geometry>
                        <sphere radius="0.025"/>
                    </geometry>
                    <material name="green">
                        <color rgba="0 1 0 1"/>
                    </material>
                </visual>
                <collision>
                    <geometry>
                        <sphere radius="0.025"/>
                    </geometry>
                </collision>
                <inertial>
                    <mass value="0.1"/>
                    <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
                </inertial>
            </link>
        </robot>
        """
    return None

def spawn_objects():
    rospy.init_node('spawn_objects_node')
    
    config = rospy.get_param('/spawn_objects')
    
    num_objects = config['number_of_objects']
    center_x = config['spawn_center']['pos_x']
    center_y = config['spawn_center']['pos_y']
    center_z = config['spawn_center']['pos_z']
    x_length = config['spawn_radius']['x_length']
    y_length = config['spawn_radius']['y_length']
    
    enabled_objects = []
    if config['object_types']['cube']:
        enabled_objects.append('cube')
    if config['object_types']['cylinders']:
        enabled_objects.append('cylinder')
    if config['object_types']['spheres']:
        enabled_objects.append('sphere')
    
    if not enabled_objects:
        rospy.logerr("No object types enabled in configuration!")
        return

    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn_model = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    for i in range(num_objects):

        x = center_x + random.uniform(-x_length, x_length)
        y = center_y + random.uniform(-y_length, y_length)
        z = center_z

        object_type = random.choice(enabled_objects)
        
        object_urdf = get_object_urdf(object_type)
        
        pose = Pose()
        pose.position = Point(x, y, z)
        pose.orientation = Quaternion(0, 0, 0, 1)

        object_name = f"{object_type}_{i}"
        try:
            spawn_model(object_name, object_urdf, "", pose, "world")
            rospy.loginfo(f"Spawned {object_type} {i} at position x={x}, y={y}, z={z}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service call failed: {e}")

        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        spawn_objects()
    except rospy.ROSInterruptException:
        pass
