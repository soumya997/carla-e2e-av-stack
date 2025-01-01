#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg


class InitialPoseToStaticTF(object):
    def __init__(self):
        # Create static transform broadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # We only want to publish the static transform once,
        # so let's store whether we've already done so.
        self.got_initial_pose = False

        # Subscribe to /initialpose (PoseWithCovarianceStamped)
        self.sub = rospy.Subscriber(
            '/initialpose',
            geometry_msgs.msg.PoseWithCovarianceStamped,
            self.initialpose_callback
        )

    def initialpose_callback(self, msg):
        # If we already published once, ignore subsequent messages.
        if self.got_initial_pose:
            return

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract orientation (a quaternion)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Build the TransformStamped
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "map"           # parent frame
        transform_stamped.child_frame_id = "camera_init"    # child frame

        # Fill in the translation
        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = z

        # Fill in the rotation
        transform_stamped.transform.rotation.x = qx
        transform_stamped.transform.rotation.y = qy
        transform_stamped.transform.rotation.z = qz
        transform_stamped.transform.rotation.w = qw

        # Publish the static transform
        self.static_broadcaster.sendTransform(transform_stamped)
        self.got_initial_pose = True

        rospy.loginfo("Published static transform [map -> camera_init] from /initialpose.")


def main():
    rospy.init_node("initialpose_to_static_tf")
    node = InitialPoseToStaticTF()
    rospy.spin()


if __name__ == "__main__":
    main()
