#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import lanelet2
from lanelet2.projection import UtmProjector

class Lanelet2Publisher:
    def __init__(self):
        rospy.init_node('lanelet2_publisher', anonymous=True)
        self.publisher_ = rospy.Publisher('lanelet2_map', MarkerArray, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        # Get the map path from the ROS parameter server.
        # If not provided, default to "maps/Town01_lanelet.osm".
        map_path = rospy.get_param('~map_path')
        
        # Load the Lanelet2 map
        projector = UtmProjector(lanelet2.io.Origin(0, 0))
        self.lanelet_map = lanelet2.io.load(map_path, projector)
        rospy.loginfo("Map loaded from: %s", map_path)

    def timer_callback(self, event):
        marker_array = MarkerArray()
        marker_id = 0  # Initialize marker ID

        for lanelet_ in self.lanelet_map.laneletLayer:
            marker = Marker()
            marker.header.frame_id = "map"  # Frame of reference
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.3
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.ns = "lanelet"
            marker.id = marker_id  # Assign unique ID

            if "subtype" not in lanelet_.attributes.keys():
                for point in lanelet2.geometry.to2D(lanelet_.leftBound):
                    marker.points.append(Point(x=point.x, y=point.y, z=0.0))
                for point in lanelet2.geometry.to2D(lanelet_.rightBound):
                    marker.points.append(Point(x=point.x, y=point.y, z=0.0))

            marker_array.markers.append(marker)
            marker_id += 1

        self.publisher_.publish(marker_array)

if __name__ == '__main__':
    try:
        node = Lanelet2Publisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Lanelet2 Publisher")
