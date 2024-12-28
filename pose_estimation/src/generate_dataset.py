#!/usr/bin/env python3

import rospy
from pose_estimation.srv import SpawnObject, DeleteObject, MoveJoints

class main():
    rospy.init_node('generate_dataset_node')
    config = rospy.get_param('/generate_dataset')
    iteratiosn = config['iterations']
    
    rospy.wait_for_service('spawn_object')
    rospy.wait_for_service('delete_object')
    rospy.wait_for_service('move_joints')
    
    spawn_service = rospy.ServiceProxy('spawn_object', SpawnObject)  
    move_joints = rospy.ServiceProxy('move_joints', MoveJoints)
    delete_service = rospy.ServiceProxy('delete_object', DeleteObject)

    joint_values = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
    response = move_joints(joint_values)

    try:
        response = spawn_service()
        if response.success:
            print(f"Spawned object with ID: {response.object_id}")
        else:
            print(f"Failed to spawn object: {response.message}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

    try:
        response = delete_service(object_id="your_object_id")
        if response.success:
            print("Object deleted successfully")
        else:
            print(f"Failed to delete object: {response.message}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


if __name__ == '__main__':
    main()