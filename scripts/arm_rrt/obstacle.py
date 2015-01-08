#!/usr/bin/env python
import rospy

from visualization_msgs.msg import Marker
#from geometry_msgs.msg import Point

class ObstacleMarker:

    def __init__(self):

        rospy.init_node("obstacle_marker")
        self.marker_pub = rospy.Publisher("obstacle_marker", Marker, queue_size=1)

        self.obstacle_marker = Marker()
        self.obstacle_marker.type = Marker.SPHERE
        self.obstacle_marker.header.frame_id = "/world"
        self.obstacle_marker.ns = "rrt"
        self.obstacle_marker.id = 4
        self.obstacle_marker.pose.orientation.w = 1
        self.obstacle_marker.action = Marker.ADD
        self.obstacle_marker.scale.x = 0.4
        self.obstacle_marker.scale.y = 0.4
        self.obstacle_marker.scale.z = 0.4
        self.obstacle_marker.color.r = 1.0
        self.obstacle_marker.color.g = 0.0
        self.obstacle_marker.color.b = 0.0
        self.obstacle_marker.color.a = 1.0

        #The initial position of the MR
        self.obstacle_marker.pose.position.x = 0.0
        self.obstacle_marker.pose.position.y = 0.6
        self.obstacle_marker.pose.position.z = 0.2


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.obstacle_marker)
            r.sleep()

if __name__ == "__main__":
    robot = ObstacleMarker()
    robot.run()