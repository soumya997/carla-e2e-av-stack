#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath:
    def __init__(self):
        rospy.init_node('odom_to_path', anonymous=True)

        # Publisher
        self.path_pub = rospy.Publisher("/loam_path", Path, queue_size=10)

        # Subscriber
        rospy.Subscriber("/odometry_from_dynamic_tf", Odometry, self.odom_callback)

        # Path object to store poses
        self.path = Path()
        self.path.header.frame_id = "map"  # Set your desired frame_id

        rospy.loginfo(f"Subscribed to /odometry_from_dynamic_tf and publishing /loam_path")

    def odom_callback(self, msg):
        # Create a PoseStamped object from the Odometry message
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Append the pose to the path
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose)

        # Publish the path
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        OdomToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Odom to Path node.")
