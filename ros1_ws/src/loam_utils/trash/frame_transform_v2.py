#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

import numpy as np
from scipy.spatial.transform import Rotation as R

def broadcast_transform(rotation_quat, relative_translation):
    rospy.init_node('camera_init_to_map_broadcaster')

    # Create a broadcaster
    br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()

    # Fill transform details
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"  # Target frame
    t.child_frame_id = "camera_init"  # Source frame

    # Set translation
    t.transform.translation.x = relative_translation[0]
    t.transform.translation.y = relative_translation[1]
    t.transform.translation.z = relative_translation[2]

    # Set rotation (quaternion)
    t.transform.rotation.x = rotation_quat[0]
    t.transform.rotation.y = rotation_quat[1]
    t.transform.rotation.z = rotation_quat[2]
    t.transform.rotation.w = rotation_quat[3]

    # Broadcast the transform
    br.sendTransform(t)
    rospy.loginfo("Broadcasting transform from camera_init to map")

    rospy.spin()

if __name__ == "__main__":
    # Source quaternion and position
    source_quat = [-1.2246467685770567e-16, 2.736341177372564e-20, 0.9999999750374586, 0.00022343921359837887]
    source_pos = [106.6408920288086, -129.417236328125, 0.0]

    # Target quaternion and position
    target_quat = [0.0033604972861870537, 0.05495717627720382, 0.002438325448758891, 0.9984800756996457]
    target_pos = [100.76752532558375, 0.2909349191390879, -12.865532436378544]

    # Convert quaternions to rotation matrices
    source_rotation = R.from_quat(source_quat).as_matrix()
    target_rotation = R.from_quat(target_quat).as_matrix()

    # Compute the relative rotation
    relative_rotation = target_rotation @ np.linalg.inv(source_rotation)

    # Compute the relative translation
    relative_translation = np.array(target_pos) - relative_rotation @ np.array(source_pos)

    print("Relative Rotation Matrix:\n", relative_rotation)
    print("Relative Translation Vector:\n", relative_translation)

    # Convert rotation matrix to quaternion
    rotation_quat = R.from_matrix(relative_rotation).as_quat()  # [x, y, z, w]

    print("Quaternion (x, y, z, w):", rotation_quat)
    print("Translation (x, y, z):", relative_translation)

    broadcast_transform(rotation_quat, relative_translation)
