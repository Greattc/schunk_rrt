#!/usr/bin/env python
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

side = 2.01

class BorderMarker:

    def __init__(self):

        rospy.init_node("border_marker")
        self.marker_pub = rospy.Publisher("border_marker", Marker, queue_size=1)

        self.border_marker = Marker()
        self.border_marker.type = Marker.LINE_STRIP
        #self.border_marker.header.stamp = rospy.get_rostime()
        self.border_marker.header.frame_id = "/world"
        self.border_marker.ns = "rrt"
        self.border_marker.id = 10
        self.border_marker.pose.orientation.w = 1
        self.border_marker.action = Marker.ADD
        self.border_marker.scale.x = 0.02
        self.border_marker.scale.y = 0.02
        self.border_marker.scale.z = 0.02

        self.border_marker.color.r = 0.2
        self.border_marker.color.g = 0.2
        self.border_marker.color.b = 0.2
        self.border_marker.color.a = 1.0

        p = Point()
        p.x = side
        p.y = -side
        p.z = 0.1
        self.border_marker.points.append(p)

        p1 = Point()
        p1.x = -side
        p1.y = -side
        p1.z = 0.1
        self.border_marker.points.append(p1)

        p2 = Point()
        p2.x = -side
        p2.y = side
        p2.z = 0.1
        self.border_marker.points.append(p2)

        p3 = Point()
        p3.x = side
        p3.y = side
        p3.z = 0.1
        self.border_marker.points.append(p3)

        '''
        p4 = Point()
        p4.x = side
        p4.y = -side
        p4.z = 0.1
        '''
        self.border_marker.points.append(p)


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.border_marker)
            r.sleep()

if __name__ == "__main__":
    border = BorderMarker()
    border.run()