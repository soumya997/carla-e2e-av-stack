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
    # Replace with your computed values

    # # Source quaternion and position
    # target_quat= [-9.835933849850075e-05, -3.1290452066124376e-05, 0.00032596024859776415, 0.9999999415481308]
    # target_pos = [0.01693921645885583, -0.0019361239704956653, 0.0006106553212857983]

    # # Target quaternion and position
    # source_quat = [1.2915154991739152e-07, 3.7550912103400815e-05, -0.00010055051711756224, 0.9999999942397528]
    # source_pos = [124.72284698486328, -198.7583770751953, 0.0017056845827028155]

    # =--------------------   [working  -> for bagfiles]
    # Source quaternion and position
    source_quat= [-9.835933849850075e-05, -3.1290452066124376e-05, 0.00032596024859776415, 0.9999999415481308]
    source_pos = [0.01693921645885583, -0.0019361239704956653, 0.0006106553212857983]

    # Target quaternion and position
    target_quat = [1.2915154991739152e-07, 3.7550912103400815e-05, -0.00010055051711756224, 0.9999999942397528]
    target_pos = [124.72284698486328, -198.7583770751953, 0.0017056845827028155]

    # =-------------------------- [V1  src->camera_init]
    # Source quaternion and position
    # source_quat= [0.016907928788746272, -0.03978255867271062, -0.04516532855978889, 0.9980438681607653]
    # source_pos = [122.67676909322651, -12.86850394366864, 8.36416201167659]

    # # Target quaternion and position
    # target_quat = [1.311667868662498e-07, 2.979992213000311e-07, -0.00018290604572674364, 0.9999999832726362]
    # target_pos = [263.1105651855469, -199.15283203125, 0.0016807365464046597]



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
