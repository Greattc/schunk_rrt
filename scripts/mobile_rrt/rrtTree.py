#!/usr/bin/env python
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TreeMarker:

    def __init__(self):

        rospy.init_node("tree_marker")
        self.marker_pub = rospy.Publisher("tree_marker", Marker, queue_size=1)

        self.tree_marker = Marker()
        self.tree_marker.type = Marker.SPHERE_LIST
        self.tree_marker.header.stamp = rospy.get_rostime()
        self.tree_marker.header.frame_id = "/world"
        self.tree_marker.ns = "rrt"
        self.tree_marker.id = 7
        self.tree_marker.pose.orientation.w = 1
        self.tree_marker.action = Marker.ADD
        self.tree_marker.scale.x = 0.02
        self.tree_marker.scale.y = 0.02
        #self.tree_marker.scale.y = 0.1

        self.tree_marker.color.r = 1.0
        self.tree_marker.color.g = 0.0
        self.tree_marker.color.b = 0.0
        self.tree_marker.color.a = 1.0

        with open("../../data/rrt_tree.dat", 'r') as f:
            for line in f:
                line = line.rstrip("\n")
                xyz = line.split("\t")[:-1]
                p = Point()
                p.x = float(xyz[0])
                p.y = float(xyz[1])
                self.tree_marker.points.append(p)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.tree_marker)
            r.sleep()

if __name__ == "__main__":
    tree = TreeMarker()
    tree.run()
