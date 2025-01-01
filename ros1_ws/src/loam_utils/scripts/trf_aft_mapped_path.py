#!/usr/bin/env python
import rospy
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TransformPathNode:
    def __init__(self):
        rospy.init_node('transform_aft_mapped_path')

        # TF2 buffer and listener for transform lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscriber to the /aft_mapped_path topic
        self.path_sub = rospy.Subscriber('/aft_mapped_path', Path, self.path_callback)

        # Publisher for the transformed path
        self.path_pub = rospy.Publisher('/aft_mapped_path_in_map', Path, queue_size=10)

        # Name of the target frame (map)
        self.target_frame = "map"

    def path_callback(self, msg):
        try:
            # Lookup the transform from "camera_init" to "map"
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,  # Target frame
                "camera_init",  # Source frame (e.g., "camera_init")
                rospy.Time(0),  # Get the latest transform
                rospy.Duration(1.0)  # Timeout
            )
        except tf2_ros.LookupException as e:
            rospy.logwarn(f"Transform lookup failed: {e}")
            return
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn(f"Transform extrapolation failed: {e}")
            return

        # Create a new Path message for the transformed path
        transformed_path = Path()
        transformed_path.header.frame_id = self.target_frame
        transformed_path.header.stamp = rospy.Time.now()

        # Transform each pose in the path
        for pose in msg.poses:
            try:
                # Transform the pose manually using the transform data
                transformed_pose = PoseStamped()
                transformed_pose.header.frame_id = self.target_frame
                transformed_pose.header.stamp = pose.header.stamp

                # Apply the translation
                transformed_pose.pose.position.x = (
                    pose.pose.position.x + transform.transform.translation.x
                )
                transformed_pose.pose.position.y = (
                    pose.pose.position.y + transform.transform.translation.y
                )
                transformed_pose.pose.position.z = (
                    pose.pose.position.z + transform.transform.translation.z
                )

                # Apply the rotation (simplified for static transforms)
                transformed_pose.pose.orientation = transform.transform.rotation

                transformed_path.poses.append(transformed_pose)
            except Exception as e:
                rospy.logwarn(f"Failed to transform pose: {e}")

        # Publish the transformed path
        self.path_pub.publish(transformed_path)
        rospy.loginfo(f"Published transformed path to {self.target_frame}")

if __name__ == '__main__':
    try:
        node = TransformPathNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
