#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf

aloam_odom = rospy.Publisher("/aft_mapped_odom", Odometry, queue_size=1)
carla_path = rospy.Publisher("/carla/ego_vehicle/trajectory_path", Path, queue_size=1)
path = Path()

tf_listener = None  # Global variable for the tf listener


def apply_transform(position, orientation, transform):
    translation, rotation = transform  # Separate the translation and rotation parts

    # Transform position
    transformed_position = [
        position.x + translation[0],
        position.y + translation[1],
        position.z + translation[2],
    ]

    # Transform orientation (quaternion multiplication)
    input_quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    transformed_quaternion = tf.transformations.quaternion_multiply(rotation, input_quaternion)

    return transformed_position, transformed_quaternion

def get_static_transform():
    try:
        # Look up the transform between camera_init and map
        trans, rot = tf_listener.lookupTransform("map", "camera_init", rospy.Time(0))
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Failed to get transform between camera_init and map!")
        return None
    


def path_callback(data):
    if not data.poses:
        rospy.logwarn("Received empty path message!")
        return

    transform = get_static_transform()
    if transform is None:
        return

    odom = Odometry()
    odom.header.frame_id = "map"
    odom.header.stamp = data.header.stamp  # Copy timestamp from incoming message

    # Apply static transform
    transformed_position, transformed_quaternion = apply_transform(
        data.poses[-1].pose.position, data.poses[-1].pose.orientation, transform
    )
    odom.pose.pose.position.x = transformed_position[0]
    odom.pose.pose.position.y = transformed_position[1]
    odom.pose.pose.position.z = transformed_position[2]
    odom.pose.pose.orientation.x = transformed_quaternion[0]
    odom.pose.pose.orientation.y = transformed_quaternion[1]
    odom.pose.pose.orientation.z = transformed_quaternion[2]
    odom.pose.pose.orientation.w = transformed_quaternion[3]

    aloam_odom.publish(odom)

def odom_callback(data):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = data.header.stamp  # Copy timestamp from incoming message
    pose.pose.position = data.pose.pose.position
    pose.pose.orientation = data.pose.pose.orientation

    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    path.poses.append(pose)

    carla_path.publish(path)


def main():
    global tf_listener

    rospy.init_node("message_change", anonymous=True)

    # Initialize the tf listener
    tf_listener = tf.TransformListener()

    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, odom_callback, queue_size=1)
    rospy.Subscriber("/aft_mapped_path", Path, path_callback, queue_size=1)

    rospy.spin()


if __name__ == "__main__":
    main()
