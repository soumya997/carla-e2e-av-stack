#!/usr/bin/env python3

# In ROS1, we use rospy instead of rclpy.
import rospy
from geometry_msgs.msg import PoseStamped

class GoalPoseRepublisher(object):
    def __init__(self, role_name):
        self.role_name = role_name
        # Subscribe to the /goal_pose topic
        self.subscription = rospy.Subscriber(
            '/move_base_simple/goal',
            PoseStamped,
            self.goal_pose_callback,
            queue_size=10
        )

        # Publisher to /carla/<ROLE_NAME>/goal
        self.publisher = rospy.Publisher(
            f'/carla/{self.role_name}/goal',
            PoseStamped,
            queue_size=10
        )
        print("Re-mapping started ...")

    def goal_pose_callback(self, msg):
        # Log the received goal pose
        rospy.loginfo(f'Received goal pose: {msg}')

        # Create a new PoseStamped message to publish
        new_msg = PoseStamped()

        # Copy the header from the received message
        new_msg.header.stamp = rospy.Time.now()  # Use the current time for the header
        new_msg.header.frame_id = 'map'  # You can change this to the appropriate frame id

        # Modify the pose values
        new_msg.pose.position.x = msg.pose.position.x
        new_msg.pose.position.y = msg.pose.position.y
        new_msg.pose.position.z = 0.2  #msg.pose.position.z  # Retain the original Z value

        # Copy the orientation directly from the received message
        new_msg.pose.orientation = msg.pose.orientation

        # Republish the goal pose to the Carla-specific topic
        self.publisher.publish(new_msg)
        rospy.loginfo(f'Published goal pose to /carla/{self.role_name}/goal')


def main():
    rospy.init_node('goal_remap', anonymous=True)

    # Replace 'my_role_name' with your actual role name in Carla
    role_name = 'ego_vehicle'

    goal_pose_republisher = GoalPoseRepublisher(role_name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
