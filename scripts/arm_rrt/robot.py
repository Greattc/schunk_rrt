#!/usr/bin/env python
import rospy

from visualization_msgs.msg import Marker
#from geometry_msgs.msg import Point

class RobotMarker:

    def __init__(self):

        rospy.init_node("robot_marker")
        self.marker_pub = rospy.Publisher("robot_marker", Marker, queue_size=1)

        self.robot_marker = Marker()
        self.robot_marker.type = Marker.SPHERE
        self.robot_marker.header.frame_id = "/world"
        self.robot_marker.ns = "rrt"
        self.robot_marker.id = 5
        self.robot_marker.pose.orientation.w = 1
        self.robot_marker.action = Marker.ADD
        self.robot_marker.scale.x = 0.03
        self.robot_marker.scale.y = 0.03
        self.robot_marker.scale.z = 0.03
        self.robot_marker.color.r = 1.0
        self.robot_marker.color.g = 0.5
        self.robot_marker.color.b = 0.0
        self.robot_marker.color.a = 1.0

        #The initial position of the MR
        '''
        self.robot_marker.pose.position.x = 0.496475
        self.robot_marker.pose.position.y = 0.554664
        self.robot_marker.pose.position.z = 0.293018
        '''
        self.robot_marker.pose.position.x = 0.328942
        self.robot_marker.pose.position.y = 0.588242
        self.robot_marker.pose.position.z = 0.415739

        self.goal_marker = Marker()
        self.goal_marker.type = Marker.SPHERE
        self.goal_marker.header.frame_id = "/world"
        self.goal_marker.ns = "rrt"
        self.goal_marker.id = 6
        self.goal_marker.pose.orientation.w = 1
        self.goal_marker.action = Marker.ADD
        self.goal_marker.scale.x = 0.03
        self.goal_marker.scale.y = 0.03
        self.goal_marker.scale.z = 0.03
        self.goal_marker.color.r = 1.0
        self.goal_marker.color.g = 0.5
        self.goal_marker.color.b = 0.0
        self.goal_marker.color.a = 1.0

        #The initial position of the MR
        self.goal_marker.pose.position.x = -0.4
        self.goal_marker.pose.position.y = 0.6
        self.goal_marker.pose.position.z = 0.3

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.robot_marker)
            self.marker_pub.publish(self.goal_marker)
            r.sleep()

if __name__ == "__main__":
    robot = RobotMarker()
    robot.run()