#!/usr/bin/env python
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class RRTPath:

    def __init__(self):

        rospy.init_node("path_plot")
        self.marker_pub = rospy.Publisher("dynamic_path", Marker, queue_size=10)

        self.node = Marker()
        self.node.type = Marker.SPHERE_LIST
        #self.node.header.stamp = rospy.get_time()
        self.node.header.frame_id = "/world"
        self.node.ns = "rrt"
        self.node.id = 0
        self.node.pose.orientation.w = 1
        self.node.action = Marker.ADD
        self.node.scale.x = 0.01
        self.node.scale.y = 0.01
        self.node.scale.z = 0.01
        self.node.color.r = 0.0
        self.node.color.g = 1.0
        self.node.color.b = 0.0
        self.node.color.a = 1.0

        self.path = Marker()
        self.path.type = Marker.LINE_STRIP
        #self.node.header.stamp = rospy.get_time()
        self.path.header.frame_id = "/world"
        self.path.ns = "rrt"
        self.path.id = 1
        self.path.pose.orientation.w = 1
        self.path.action = Marker.ADD
        self.path.scale.z = 0.005
        self.path.scale.x = 0.005
        self.path.scale.y = 0.005
        self.path.color.r = 0.0
        self.path.color.g = 0.0
        self.path.color.b = 1.0
        self.path.color.a = 1.0

        with open("../../data/dynamic_point.dat", 'r') as f:
            for line in f:
                line = line.rstrip("\n")
                xyz = line.split("\t")[:-1]
                p = Point()
                p.x = float(xyz[0])
                p.y = float(xyz[1])
                p.z = float(xyz[2])
                self.path.points.append(p)
                self.node.points.append(p)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #self.marker_pub.publish(self.path)
            self.marker_pub.publish(self.node)
            r.sleep()

if __name__=="__main__":
    path = RRTPath()
    path.run()
