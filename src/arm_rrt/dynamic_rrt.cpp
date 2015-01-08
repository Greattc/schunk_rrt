#include "schunk_rrt/dynamic_rrt.h"
#include <ros/package.h>

/*
 ***********************
 ***** Dynamic RRT *****
 ***********************
*/

static const std::string PACKAGE = "schunk_rrt";

DynamicRRT::DynamicRRT(NodePtr init, NodePtr goal, ros::NodeHandle &nh1, ros::NodeHandle &nh2):init_flag(false),current_index(0),plan_index(0)
{	
    initialize();
	init_ = init;
	goal_ = goal;
    rrt = RRTPtr(new RRT(init_, goal_));
    n = nh1;
    n2 = nh2;
    current_index = 1;

//    marker_publisher = n.advertise<visualization_msgs::Marker>("path_marker", 3);
    path_publisher = n2.advertise<schunk_rrt::RRTPath>("rrt_path_array", 1);

    obstacle_sub_ =  n.subscribe("dynamic_obstacle", 1, &DynamicRRT::obstacle_cb, this);
}

void DynamicRRT::initialize()
{
    //RViz中Marker的基本设置
    node.header.frame_id = line.header.frame_id = "/world";
    node.header.stamp = line.header.stamp = ros::Time::now();
    node.ns = line.ns = "rrt";
    node.action = line.action = visualization_msgs::Marker::ADD;
    node.pose.orientation.w = line.pose.orientation.w = 1.0;
    node.id = 0;
    line.id = 1;
    node.type = visualization_msgs::Marker::SPHERE_LIST;
    line.type = visualization_msgs::Marker::LINE_STRIP;

    line.scale.x = 0.01;
    line.scale.y = 0.01;
    //蓝色线条
    line.color.b = 1.0;
    line.color.a = 1.0;

    node.scale.x = 0.02;
    node.scale.y = 0.02;
    //绿色节点
    node.color.g = 1.0;
    node.color.a = 1.0;
}

void DynamicRRT::run()
{
}

bool DynamicRRT::checkCurrentPath()
{
    //当前运动到的位置
    int end_index = path_size - current_index;
    ROS_INFO("End Index: %d", end_index);
    for(int i=0; i<end_index; ++i)
    {
//        std::cout << rrt->straj[end_index-i-1][0] << std::endl;
        if(rrt->checkCollisions(rrt->straj[end_index-i-1]))
        {
            std::cout << rrt->straj[end_index-i-1][0] << std::endl;
            ROS_INFO("Current path is collisioning, recomputing...");
            collision_index = end_index - i;
            return true;
        }
    }

    ROS_INFO("Current path is no collisioning, continuing...");
    return false;
}

void DynamicRRT::segmentRRT()
{
    static double time_node = 0.01;
//    int plan_index = traj.size()-current_index - 5;
    //Give some space to response
    plan_index = collision_index + 20;

    //未发生碰撞的位置分别在之前和之后的节点
    NodePtr newInit(new Node(rrt->straj[plan_index]));

    RRTPtr rrt2(new RRT( newInit, rrt->getGoal()));

    //rrt2的障碍物参数为默认参数，必须跟rrt的一致
    for(int i=0; i<rrt->collisionGroup.size(); ++i)
        rrt2->addObstacle(rrt->collisionGroup[i], rrt->collisionTFGroup[i]);

//    ROS_INFO("Current Index: %d", path_size - current_index);
    ROS_INFO("Collision Index: %d", collision_index);
    ROS_INFO("Start Planning Node: %d", plan_index);
//    std::cout << rrt->straj[plan_index] << std::endl;

    //rrt2开始规划
    rrt2->run(false);

    if(rrt2->getState())
    {
        rrt2->traj.erase(rrt2->traj.end()-1, rrt2->traj.end());
        writePath(rrt2);

        vector<double> init_vec(7);
//        init_vec = (rrt->straj[plan_index]-rrt->straj[plan_index+1])*(1/time_node);
        init_vec = (rrt->straj[plan_index+1]-rrt->straj[plan_index])*(1/time_node);
        std::cout << "init_vec:\n " << init_vec << std::endl;

        vector<double> end_vec(7,0.0);
        vector<double> current_vec(7);

        int size_rrt2 = rrt2->traj.size();
        current_vec = (rrt2->traj[size_rrt2-1] - rrt2->traj[size_rrt2-2])*(1/(4*time_node));
        std::cout << "current_vec:\n " << current_vec << std::endl;

        if(vectorEqual(current_vec, init_vec))
        {
            rrt2->smooth(init_vec, end_vec, false);
        }
        else
            rrt2->smooth(init_vec, end_vec, true, 0.2);

        ROS_INFO("Inside RRT path size: %ld", rrt2->straj.size());

//        path_size = path_size - plan_index + rrt2->straj.size() ;
//        ROS_INFO("The whole RRT path size: %d", path_size);
        pathPublish(rrt2);

        /*
        //旧路径碰撞段的删除和新路径的添加
        Path::iterator cbegin = rrt->mpath.begin()+rrt->cPos_[1]+2;

        Trajectory::iterator tbegin = rrt->traj.begin()+rrt->cPos_[1]+2;

        rrt->mpath.erase(rrt->mpath.begin(), cbegin);
        rrt->traj.erase(rrt->traj.begin(), tbegin);
        //删除以后能够确定现在轨迹无碰撞
        //std::cout<<collisionPos(rrt)<<std::endl;
//        std::cout<<"Segment"<<"\t"<<rrt->mpath.size()<<std::endl;
//        std::cout<<"Segment"<<"\t"<<rrt2->mpath.size()<<std::endl;

        std::cout<<"Segment"<<"\t"<<rrt->traj.size()<<std::endl;
        std::cout<<"Segment"<<"\t"<<rrt2->traj.size()<<std::endl;
        //检查是否修剪后以及新增轨迹碰撞
        //std::cout<<collisionPos(rrt)<<std::endl;
        //std::cout<<collisionPos(rrt2)<<std::endl;

        //trajectory::iterator newbegin = rrt->path.begin()+rrt->cPos_[0]-1;
        rrt->mpath.insert(rrt->mpath.begin(), rrt2->mpath.begin(), rrt2->mpath.end());
        rrt->traj.insert(rrt->traj.begin(), rrt2->traj.begin(), rrt2->traj.end());
        //rrt->mpath.insert(rrt->mpath.begin(), rrt2->mpath.begin(), rrt2->mpath.end());

//        std::cout<<"Segment"<<"\t"<<rrt->mpath.size()<<std::endl;
        std::cout<<"Segment"<<"\t"<<rrt->traj.size()<<std::endl;
        //std::cout<<rrt->obstacle[0]<<"\t"<<rrt->obstacle[1]<<std::endl;
        //std::cout<<rrt2->obstacle[0]<<"\t"<<rrt2->obstacle[1]<<std::endl;

        //修剪重规划以后的轨迹无碰撞，但是碰撞起始末端位置仍为修剪前的位置
        //std::cout<<collisionPos(rrt)<<std::endl;
        //std::cout<<rrt->cPos_[0]<<"\t"<<rrt->cPos_[1]<<std::endl;
        */
    }
}

void DynamicRRT::pathPublish(const RRTPtr &rrt)
{
    path_msg_.path.clear();

    path_msg_.breakPoint = path_size - plan_index;
    int traj_size = rrt->straj.size();
    for(int i=0; i<traj_size; ++i)
    {
        schunk_rrt::RRTPoint p;
        p.point.resize(7);

        for(int j=0; j<7; ++j)
           p.point[j] = rrt->straj[traj_size-1-i][j];

        path_msg_.path.push_back(p);
    }
    ROS_INFO("Break Point: %d", path_msg_.breakPoint);
    path_publisher.publish(path_msg_);
}

void DynamicRRT::markerMake()
{
    line.points.clear();
    node.points.clear();

    using namespace std;
//    cout << rrt->mpath.size() << endl;
    for(int j=0; j<rrt->mpath.size(); ++j)
    {
        geometry_msgs::Point p;
        p.x = rrt->mpath[j][0];
        p.y = rrt->mpath[j][1];
        p.z = rrt->mpath[j][2];

        line.points.push_back(p);
        node.points.push_back(p);
    }
}

void DynamicRRT::writeFile(const RRTPtr &rrt)
{
    std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream rrtJoints( (filepath+"/data/dynamic_traj.dat").c_str());

    int size_p = rrt->straj.size();
    for(int i=0; i<size_p; ++i)
    {
        for(int j=0; j<rrt->straj[0].size(); ++j)
        {
            rrtJoints<<rrt->straj[size_p - i - 1][j]<<"\t";
        }
        rrtJoints<<"\n";
    }

    rrtJoints.close();
}

void DynamicRRT::writePath(const RRTPtr &rrt)
{
    std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream rrtPath( (filepath+"/data/rrt_path.dat").c_str());

    int size_p = rrt->traj.size();
    for(int i=0; i<size_p; ++i)
    {
        for(int j=0; j<rrt->traj[0].size(); ++j)
        {
            rrtPath<<rrt->traj[size_p - i - 1][j]<<"\t";
        }
        rrtPath<<"\n";
    }

    rrtPath.close();
}

void DynamicRRT::publishMarker()
{
    markerMake();

    path_publisher.publish(line);
    path_publisher.publish(node);
}


void DynamicRRT::obstacle_cb(const visualization_msgs::MarkerPtr& msg)
{
    fcl::Vec3f t(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    rrt->collisionTFGroup[0].setTranslation(t);

    if(!init_flag)
    {
        init_flag = true;
        ROS_INFO("First time to run RRT");
        rrt->run(false);
        vector<double> init_vec(7,0.0);
        vector<double> end_vec(7,0.0);

        rrt->smooth(init_vec, end_vec);
        path_size = rrt->straj.size();
//        writeFile(rrt);

        ROS_INFO("Initial Path Size: %d", path_size);

        //获取开始运动时间
        gettimeofday(&start, NULL);
        start_t = start.tv_sec+(start.tv_usec/1000000.0);

        pathPublish(this->rrt);
    }
    else
    {
        using namespace std;

        gettimeofday(&start, NULL);
        double now_t = start.tv_sec+(start.tv_usec/1000000.0);
        ROS_INFO("From beginning time elapsed: %.6lfs\n", now_t - start_t);

        current_index = int(ceil((now_t - start_t)*frequency)) + threshold;
        if( (current_index < path_size) && checkCurrentPath())
        {
            ROS_INFO("Current Index: %d", current_index);
            segmentRRT();
        }
    }
}
