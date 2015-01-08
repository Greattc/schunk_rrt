#ifndef schunk_rrt_DYNAMIC_RRT_H_
#define schunk_rrt_DYNAMIC_RRT_H_

#include "schunk_rrt/arm_rrt.h"
#include <fstream>
#include <cmath>
#include <sys/time.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <schunk_rrt/RRTPath.h>
#include <schunk_rrt/RRTPoint.h>

#define frequency 50.0
#define threshold 0

class DynamicRRT
{
public:
//    DynamicRRT(NodePtr init, NodePtr goal);
    DynamicRRT(NodePtr init, NodePtr goal, ros::NodeHandle &nh1, ros::NodeHandle &nh2);
    void run();

    void writeFile(const RRTPtr &rrt);
    void writePath(const RRTPtr &rrt);

	ros::NodeHandle n;
    ros::NodeHandle n2;

    ros::Subscriber obstacle_sub_;

    ros::Publisher path_publisher;
    ros::Publisher marker_publisher;

    //RRT节点及连线轨迹Marker
    visualization_msgs::Marker line, node;

    RRTPtr rrt;

    schunk_rrt::RRTPath path_msg_;

    bool init_flag;

private:
    void initialize();
    void markerMake();
    void segmentRRT();

    void obstacle_cb(const visualization_msgs::MarkerPtr& joy_msg);
    void pathPublish(const RRTPtr &rrt);
    bool checkCurrentPath();

    void publishMarker();

    NodePtr init_;
    NodePtr goal_;

    struct timeval start;
    double start_t;


    //当前运动到达的位置
    int current_index;
    int collision_index;
    int plan_index;
    int path_size;
};

#endif
