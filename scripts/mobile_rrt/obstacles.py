#!/usr/bin/env python
import rospy
import random

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ObstacleMarker:

    def __init__(self):

        rospy.init_node("obstacle_marker")
        self.marker_pub = rospy.Publisher("obstacle_marker", Marker, queue_size=1)

        self.obstacle_marker = Marker()
        self.obstacle_marker.type = Marker.CUBE_LIST
        #self.obstacle_marker.header.stamp = rospy.get_time()
        self.obstacle_marker.header.frame_id = "/world"
        self.obstacle_marker.ns = "rrt"
        self.obstacle_marker.id = 2
        self.obstacle_marker.pose.orientation.w = 1
        self.obstacle_marker.action = Marker.ADD
        self.obstacle_marker.scale.x = 0.2
        self.obstacle_marker.scale.y = 0.2
        self.obstacle_marker.scale.z = 0.2
        self.obstacle_marker.color.r = 0.0
        self.obstacle_marker.color.g = 0.0
        self.obstacle_marker.color.b = 1.0
        self.obstacle_marker.color.a = 1.0

        obstacle_pos = open("../../data/obstacle_position.dat", 'w')

        for i in range(30):
            p = Point()
            p.x = random.randint(-10, 9) * 0.2 + 0.1
            p.y = random.randint(-10, 9) * 0.2 + 0.1
            p.z = 0.1
            if p.x != 0.1 and p.y != 0.1:
                obstacle_pos.write(str(p.x)+'\t')
                obstacle_pos.write(str(p.y)+'\n')
                self.obstacle_marker.points.append(p)

        obstacle_pos.close()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.obstacle_marker)
            r.sleep()

if __name__ == "__main__":
    obstacle = ObstacleMarker()
    obstacle.run()
