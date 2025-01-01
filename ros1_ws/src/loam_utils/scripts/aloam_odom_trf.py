#!/usr/bin/env python
import rospy
import tf2_ros
from nav_msgs.msg import Odometry

class DynamicTransformToOdometry:
    def __init__(self):
        rospy.init_node('dynamic_transform_to_odometry')

        # TF2 buffer and listener for transform lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publisher for the Odometry message
        self.odom_pub = rospy.Publisher('/odometry_from_dynamic_tf', Odometry, queue_size=10)

        # Names of the frames
        self.target_frame = "camera_init"
        self.source_frame = "aft_mapped"

        # Start publishing odometry
        self.publish_odometry()

    def publish_odometry(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                # Lookup the dynamic transform from "aft_mapped" to "camera_init"
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,  # Target frame
                    self.source_frame,  # Source frame
                    rospy.Time(0)  # Get the latest transform
                )

                # Create an Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = self.target_frame
                odom_msg.child_frame_id = self.source_frame

                # Fill in the pose (position and orientation)
                odom_msg.pose.pose.position.x = transform.transform.translation.x
                odom_msg.pose.pose.position.y = transform.transform.translation.y
                odom_msg.pose.pose.position.z = transform.transform.translation.z
                odom_msg.pose.pose.orientation = transform.transform.rotation

                # For simplicity, velocity values are left zeroed out
                odom_msg.twist.twist.linear.x = 0.0
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0
                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = 0.0

                # Publish the Odometry message
                self.odom_pub.publish(odom_msg)
                rospy.loginfo("Published Odometry from dynamic transform")

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Failed to lookup dynamic transform: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        node = DynamicTransformToOdometry()
    except rospy.ROSInterruptException:
        pass
