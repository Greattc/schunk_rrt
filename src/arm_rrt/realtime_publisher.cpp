#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <schunk_rrt/RRTPoint.h>
#include <schunk_rrt/RRTPath.h>
#include <ros/callback_queue.h>
#include <schunk_kinematics/arm_kinematics.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

#define NUMBER_OF_JOINTS 7 //<-- SCHUNK 7 JOINTS!
static double cycle = 0.02;  //控制周期（插值运动采样时间）

static const std::string PACKAGE = "schunk_rrt";

using namespace std;

class RealtimePublisher
{
public:
    RealtimePublisher(ros::NodeHandle &nh1, ros::NodeHandle &nh2);
    ~RealtimePublisher();
    void publish();
	void run();
    void writeLog();

private:
	
    ros::NodeHandle n;
    ros::NodeHandle n2;

    ros::Timer timer;

    vector<string> joints_controller_names;
    vector<vector<double> > robot_traj;
    int running_node;

    bool init_flag;
 
    void path_cb(const schunk_rrt::RRTPathPtr &msg);
    void timer_cb(const ros::TimerEvent& event);

    vector<ros::Publisher> desired_joints_values_publishers;
    ros::Subscriber sub;

    Eigen::VectorXd linkLen;

    //动态末端位置变化
    ofstream rrtPoint;
    //动态关节角度变化
    ofstream rrtJoint;

};

RealtimePublisher::RealtimePublisher(ros::NodeHandle &nh1, ros::NodeHandle &nh2):init_flag(false),running_node(0),n(nh1),n2(nh2)
{

    linkLen.resize(NUMBER_OF_JOINTS);
    linkLen<<0.3,0.0,0.328,0.0,0.276,0.0,0.1785;  //No SDH

    string filepath = ros::package::getPath(PACKAGE);
    rrtPoint.open((filepath+"/data/dynamic_point.dat").c_str());
    rrtJoint.open((filepath+"/data/dynamic_joint.dat").c_str());

    joints_controller_names.push_back("joint1_position_controller");
    joints_controller_names.push_back("joint2_position_controller");
    joints_controller_names.push_back("joint3_position_controller");
    joints_controller_names.push_back("joint4_position_controller");
    joints_controller_names.push_back("joint5_position_controller");
    joints_controller_names.push_back("joint6_position_controller");
    joints_controller_names.push_back("joint7_position_controller");

    desired_joints_values_publishers.resize(NUMBER_OF_JOINTS);
    for(unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i){
        //string topic = name_space + "/" + joints_controller_names[i] + "/" + comand;
        string topic = joints_controller_names[i] + "/" + "command";   
        desired_joints_values_publishers[i] = n.advertise<std_msgs::Float64>(topic, 1);
    }

    sub = n.subscribe("/rrt_path_array", 1, &RealtimePublisher::path_cb, this);

    ROS_INFO("Creating a timer to command controller!");
    timer = n2.createTimer(ros::Duration(cycle),&RealtimePublisher::timer_cb, this);
}

RealtimePublisher::~RealtimePublisher()
{
    writeLog();
    rrtPoint.close();
    rrtJoint.close();
}

void RealtimePublisher::publish()
{
    Eigen::VectorXd seta(7);
    for(int i=0; i < NUMBER_OF_JOINTS; ++i)
    {
        std_msgs::Float64 msg;
        msg.data = robot_traj[i][running_node];
        seta(i) = robot_traj[i][running_node];

        desired_joints_values_publishers[i].publish(msg);
    }
    ++running_node;
}

void RealtimePublisher::path_cb(const schunk_rrt::RRTPathPtr &msg)
{
    ROS_INFO("New trajectory comming!");
    ROS_INFO("Break Point: %d", msg->breakPoint);
    if(!init_flag)
    {
        robot_traj.resize(NUMBER_OF_JOINTS);
        for(int i=0; i<NUMBER_OF_JOINTS; ++i) 
        {
            robot_traj[i].resize(msg->path.size());
            for(int k=0; k<msg->path.size(); ++k)
                robot_traj[i][k] = msg->path[k].point[i];
        }

        init_flag = true;
    }
    else
    {
        ROS_INFO("Path Size: %ld", msg->path.size());
        vector<vector<double> > next_traj;

        next_traj.resize(NUMBER_OF_JOINTS);
        for(int i=0; i<NUMBER_OF_JOINTS; ++i) 
        {
            next_traj[i].resize(msg->path.size());
            for(int k=0; k<msg->path.size(); ++k)
                next_traj[i][k] = msg->path[k].point[i];
        }

        /*
        vector<double>::iterator it;
        it = find(robot_traj[0].begin(), robot_traj[0].end(), next_traj[0][0]);
        int start = (int)(it-robot_traj[0].begin());
        for(int i=0; i<NUMBER_OF_JOINTS; ++i) 
        {
            robot_traj[i].erase(robot_traj[i].begin()+start, robot_traj[i].end());
            robot_traj[i].insert(robot_traj[i].end(), next_traj[i].begin(),next_traj[i].end());
        }
        */

        for(int i=0; i<NUMBER_OF_JOINTS; ++i)
        {
            robot_traj[i].erase(robot_traj[i].begin()+msg->breakPoint, robot_traj[i].end());
//            robot_traj[i].erase(robot_traj[i].begin()+msg->breakPoint-1, robot_traj[i].end());
            robot_traj[i].insert(robot_traj[i].end(), next_traj[i].begin()+1,next_traj[i].end());
        }
    }
}

void RealtimePublisher::timer_cb(const ros::TimerEvent& event)
{
//    ROS_INFO("Inside the timer.");
    if(init_flag && running_node < robot_traj[0].size())
    {
//        ROS_INFO("Path: %d  / %ld", running_node, robot_traj[0].size());
//        ROS_INFO("Path: %f", robot_traj[0][running_node]);
        publish();
    }
}

void RealtimePublisher::run()
{
    if(init_flag)
        publish();
}

void RealtimePublisher::writeLog()
{
    Eigen::VectorXd seta(7);
    for(int k=0; k<robot_traj[0].size(); ++k)
    {
        for(int i=0; i < NUMBER_OF_JOINTS; ++i)
        {
            seta(i) = robot_traj[i][k];
            rrtJoint << seta(i) << "\t";
        }

        rrtJoint<<"\n";

        //记录动态避障过程中末端位置的变化
        Eigen::MatrixXd eef = forward_kinematics(seta, linkLen);
        rrtPoint<< eef(0,3) << "\t" << eef(1,3) << "\t" << eef(2,3) << "\t" << "\n";

    }
}


int main(int argc, char **argv)
{
    ROS_INFO("NODE TO SEND TRAJECTORIES...START!");
    //**** INITIALIZATION OF ROS, MSGS AND PUBLISHERS ****
    ros::init(argc, argv, "realtime_publisher");

    ros::NodeHandle nh1;

    //second nodehandle and service queue for working in second thread
    ros::NodeHandle nh2(nh1);
    ros::CallbackQueue timer_queue(true);
    nh2.setCallbackQueue(&timer_queue);

    //pass both nodehandles
    RealtimePublisher rtp(nh1,nh2);
    ros::Rate r(50);

    //start threads with different callback queue: the first one handles the global one
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::AsyncSpinner spinner2(1,&timer_queue);
    spinner2.start();

    ros::waitForShutdown();

    return 0;
}
