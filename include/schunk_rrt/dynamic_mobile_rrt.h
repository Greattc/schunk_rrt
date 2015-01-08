#include "schunk_rrt/mobile_rrt.h"

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

class DynamicRRT
{
public:
    DynamicRRT(NodePtr init, NodePtr goal);
    void run();

private:
    void initialize();
    bool checkCol(RRTPtr &rrt);
    void segmentRRT(RRTPtr &rrt);

    void writeFile(const RRTPtr &rrt);
    void markerMake();
    void moveCurrentNode();
    bool checkSingleCollision(Coord &coord);
    void callback(const visualization_msgs::MarkerPtr &msg);

    RRTPtr rrt;
    ros::NodeHandle n;
    ros::Publisher path_publisher;
    ros::Subscriber collision_listener;

    visualization_msgs::Marker line;
    visualization_msgs::Marker node;

    //当前运动到达的位置
    int current_pos;

    bool isFinished;
    NodePtr init_;
    NodePtr goal_;
};
