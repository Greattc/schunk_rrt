#include "schunk_rrt/dynamic_mobile_rrt.h"
#include <ros/package.h>

#include <iterator>
#include <iostream>
#include <fstream>

using std::cout;
using std::endl;

static const std::string PACKAGE = "schunk_rrt";

/*
 ***********************
 ***** Dynamic RRT *****
 ***********************
*/

DynamicRRT::DynamicRRT(NodePtr init, NodePtr goal)
{
    initialize();
	init_ = init;
	goal_ = goal;
    rrt = RRTPtr(new RRT(init_, goal_));
    rrt->run(false);
    markerMake();

    isFinished = false;
    current_pos = 1;
    collision_listener = n.subscribe("obstacle_marker", 10, &DynamicRRT::callback, this);
    path_publisher = n.advertise<visualization_msgs::Marker>("dynamic_rrt",1);
    path_publisher.publish(node);
    path_publisher.publish(line);
}

void DynamicRRT::initialize()
{
    //RViz中Marker的基本设置
    node.header.frame_id = line.header.frame_id = "/world";
    node.header.stamp = line.header.stamp = ros::Time::now();
    node.ns = line.ns = "rrt";
    node.action = line.action = visualization_msgs::Marker::ADD;
    node.pose.orientation.w = line.pose.orientation.w = 1.0;
    node.id = 8;
    line.id = 9;

    node.type = visualization_msgs::Marker::SPHERE_LIST;
    line.type = visualization_msgs::Marker::LINE_STRIP;

    line.scale.x = 0.02;
    line.scale.y = 0.02;
    //蓝色线条
    line.color.g = 1.0;
    line.color.r = 1.0;
    line.color.a = 1.0;

    node.scale.x = 0.02;
    node.scale.y = 0.02;
    //绿色节点
    node.color.g = 1.0;
    node.color.a = 1.0;
}

bool DynamicRRT::checkSingleCollision(Coord &coord)
{
    Coord current_coord = rrt->path[current_pos];
    Coord goal_coord = goal_->getCoord();

    double temp = robot_radius * 2;

    if((fabs(coord[0] - current_coord[0])<temp && fabs(coord[1] -current_coord[1])<temp)
            || ((fabs(coord[0] - goal_coord[0])<temp && fabs(coord[1] - goal_coord[1])<temp)))
        return true;
    return false;
}

void DynamicRRT::callback(const visualization_msgs::MarkerPtr &msg)
{
    ROS_INFO("Listening...");
    moveCurrentNode();
    if(!isFinished && current_pos < rrt->path.size())
    {
        //更新障碍物信息
        rrt->clearObstacle();
        for(unsigned int i=0; i<msg->points.size(); ++i)
        {
            Coord coord(2);
            coord[0] = msg->points[i].x;
            coord[1] = msg->points[i].y;

            if(!checkSingleCollision(coord))
            {
                rrt->setParams(coord);
            }
        }
        //如果之前的轨迹发生碰撞，则修剪再生
        if(checkCol(rrt))
        {
            ROS_INFO("Need to recompute path");
            segmentRRT(rrt);
        }
        markerMake();

//        writeFile(rrt);
//        isFinished = true;
    }
    else
    {
        ROS_INFO("The RRT Search is finished! ");
//        writeFile(rrt);
//        isFinished = true;
    }
    path_publisher.publish(node);
    path_publisher.publish(line);
}

//void DynamicRRT::run()
//{
//    if(current_pos < rrt->path.size())
//    {
//        collisionControl();

//        recycleRRT(rrt);

//        current_marker.pose.position.x = rrt->traj[rrt->path.size() - current_pos][0];
//        current_marker.pose.position.y = rrt->traj[rrt->path.size() - current_pos][1];
//        current_marker.pose.position.z = rrt->traj[rrt->path.size() - current_pos][2];

//    }
//    else
//    {
//        writeFile(rrt);
//        current_marker.pose.position.x = rrt->traj.front()[0];
//        current_marker.pose.position.y = rrt->traj.front()[1];
//        current_marker.pose.position.z = rrt->traj.front()[2];

//        path_publisher.publish(line);
//        path_publisher.publish(node);
//        path_publisher.publish(collision_marker);
//        path_publisher.publish(current_marker);
//    }
//}

//控制当前节点的运动
void DynamicRRT::moveCurrentNode()
{
//    current_pos += 0.01;
    if(current_pos > rrt->path.size())
        current_pos = rrt->path.size();
    cout << current_pos << endl;
}

bool DynamicRRT::checkCol(RRTPtr &rrt)
{
    for(int i=0; i < rrt->path.size()-current_pos; ++i)
    {
        bool collision = rrt->checkCollisions(rrt->path[i]);

        if(collision) return true;
    }
    return false;
}

void DynamicRRT::segmentRRT(RRTPtr &rrt)
{
    //未发生碰撞的位置分别在之前和之后的节点
    NodePtr newInit(new Node(rrt->path[rrt->path.size() - current_pos - 1]));
//    NodePtr newInit(new Node(rrt->path[rrt->path.size() - current_pos - 1]));
//    RRTPtr rrt2(new RRT(newInit, newGoal));

    RRTPtr rrt2(new RRT( newInit, rrt->getGoal()));
    rrt2->setObstacle(rrt->getObstacle());
    //rrt2的障碍物参数为默认参数，必须跟rrt的一致
    rrt2->run(false);

    if(rrt2->getState())
    {
        //        std::cout<<"Segment"<<"\t"<<rrt->path.size()<<std::endl;

        //原来轨迹的碰撞位置

        //旧路径碰撞段的删除和新路径的添加
        trajectory::iterator cbegin = rrt->path.begin() + rrt->path.size() - current_pos;

        rrt->path.erase(rrt->path.begin(), cbegin);

        //删除以后能够确定现在轨迹无碰撞
        //std::cout<<collisionPos(rrt)<<std::endl;

//        std::cout<<"Segment"<<"\t"<<rrt->path.size()<<std::endl;
//        std::cout<<"Segment"<<"\t"<<rrt2->path.size()<<std::endl;

        //检查是否修剪后以及新增轨迹碰撞
        //std::cout<<collisionPos(rrt)<<std::endl;
        //std::cout<<collisionPos(rrt2)<<std::endl;

        rrt->path.insert(rrt->path.begin(), rrt2->path.begin(), rrt2->path.end());

//        std::cout<<"Segment"<<"\t"<<rrt->path.size()<<std::endl;
        //std::cout<<rrt->obstacle[0]<<"\t"<<rrt->obstacle[1]<<std::endl;
        //std::cout<<rrt2->obstacle[0]<<"\t"<<rrt2->obstacle[1]<<std::endl;

        //修剪重规划以后的轨迹无碰撞，但是碰撞起始末端位置仍为修剪前的位置
        //std::cout<<collisionPos(rrt)<<std::endl;
        //std::cout<<rrt->cPos_[0]<<"\t"<<rrt->cPos_[1]<<std::endl;
    }
}

void DynamicRRT::markerMake()
{
    line.points.clear();
    node.points.clear();
    for(int j=0; j<rrt->path.size(); ++j)
    {
        geometry_msgs::Point p;
        p.x = rrt->path[j][0];
        p.y = rrt->path[j][1];

        line.points.push_back(p);
        node.points.push_back(p);
    }
}

void DynamicRRT::writeFile(const RRTPtr &rrt)
{
    std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream rrtPoints( (filepath+"/data/dynamic_coords.dat").c_str());

    size_t size_p = rrt->path.size();
    for(size_t i=0; i<size_p; ++i)
    {
        for(size_t j=0; j<rrt->path[0].size(); ++j)
        {
            rrtPoints<<rrt->path[size_p - i - 1][j]<<"\t";
        }
        rrtPoints<<"\n";
    }
    rrtPoints.close();
}
