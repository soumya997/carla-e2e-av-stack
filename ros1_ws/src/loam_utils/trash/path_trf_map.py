import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_geometry_msgs import do_transform_pose

def callback(path_msg):
    transformed_path = Path()
    transformed_path.header = path_msg.header
    transformed_path.header.frame_id = 'map'  # Set the new frame

    try:
        # Lookup the transform from 'camera_init' to 'map'
        transform = tf_buffer.lookup_transform(
            'map',
            'camera_init',
            rospy.Time(0),  # Use latest available transform
            rospy.Duration(1.0))  # Timeout for lookup
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Transform lookup failed.")
        return

    for pose_stamped in path_msg.poses:
        # Create a PoseStamped message in camera_init frame
        pose_camera = PoseStamped()
        pose_camera.header = pose_stamped.header
        pose_camera.header.frame_id = 'camera_init'
        pose_camera.pose = pose_stamped.pose

        try:
            # Transform the pose to map frame
            pose_map = do_transform_pose(pose_camera, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Pose transformation failed.")
            continue

        # Add the transformed pose to the new path
        transformed_path.poses.append(pose_map)

    # Publish the transformed path
    pub.publish(transformed_path)

if __name__ == '__main__':
    rospy.init_node('path_transformer')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Subscriber to the original path
    rospy.Subscriber('/aft_mapped_path', Path, callback)

    # Publisher for the transformed path
    pub = rospy.Publisher('/aft_mapped_path_in_map', Path, queue_size=10)

    rospy.spin()