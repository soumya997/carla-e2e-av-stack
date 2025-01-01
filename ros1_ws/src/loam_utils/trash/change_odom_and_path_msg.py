#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


aloam_odom = rospy.Publisher("/aft_mapped_odom", Odometry, queue_size=1)
carla_path = rospy.Publisher("/carla/ego_vehicle/trajectory_path", Path, queue_size=1)
path = Path()


def path_callback(data):

    if not data.poses:
        rospy.logwarn("Received empty path message!")
        return
    

    odom = Odometry()
    odom.header.frame_id = "map"
    odom.header.stamp = data.header.stamp  # Copy timestamp from incoming message
    odom.pose.pose.position = data.poses[-1].pose.position
    odom.pose.pose.orientation = data.poses[-1].pose.orientation

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

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('message_change', anonymous=True)

    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, odom_callback, queue_size=1)
    rospy.Subscriber("/aft_mapped_path", Path, path_callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()