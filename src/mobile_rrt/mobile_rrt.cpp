#include "schunk_rrt/mobile_rrt.h"
#include <ros/ros.h>
#include <ros/package.h>

//标准头文件
#include <fstream>

using std::cout;
using std::endl;

static const std::string PACKAGE = "schunk_rrt";

Coord operator-(const Coord &v1, const Coord &v2)
{
    Coord result(v1);
    for (int i=0; i < v1.size(); ++i)
    {
        result[i] = v1[i] - v2[i];
    }
    return result;
}

Coord operator+(const Coord &v1, const Coord &v2)
{
    Coord result(v1);
    for (int i=0; i < v1.size(); ++i)
    {
        result[i] = v1[i] + v2[i];
    }
    return result;
}

Coord operator*(const Coord &v1, double scale)
{
    Coord result(v1);
    for (int i=0; i < v1.size(); ++i)
    {
        result[i] = v1[i] * scale;
    }
    return result;
}

Node::Node()
{
    pos_.push_back( double(RANDNM(-5.0, 5.0)) );
    pos_.push_back( double(RANDNM(-5.0, 5.0)) );
    pos_.push_back( double(RANDNM(-M_PI, M_PI)) );
    parent_ = 0;
}

Node::Node(const Coord &vec)
{
    pos_ = vec;
    parent_ = -1;
}

Node::Node(const Coord &vec, int parent)
{
    pos_ = vec;
    parent_ = parent;
}

Coord Node::getCoord()
{
    return pos_;
}

void Node::setParent(int parent)
{
    parent_ = parent;
}

RRT::RRT(NodePtr init, NodePtr goal)
{
    initialize(init, goal);
}

RRT::~RRT()
{
    cleanup();
}

void RRT::initialize(NodePtr init, NodePtr goal)
{
    rrtTree_.push_back(init);
    bestDelta = 100000;
    bestID = 0;
    count = 0;
    state = STEP_PROGRESS;
    init_ = init;
    goal_ = goal;
    kdTree = kd_create(3);
    addNode(init);
}

vector<Coord> RRT::getObstacle()
{
    return obstacle;
}

void RRT::setObstacle(const vector<Coord>& obstacle_)
{
    obstacle = obstacle_;
}

NodePtr RRT::getGoal() const
{
    return goal_;
}

void RRT::setParams(Coord vec)
{
    obstacle.push_back(vec);
}

void RRT::clearObstacle()
{
    obstacle.clear();
    obstacle.resize(0);
}

void RRT::cleanup()
{
    kd_free(kdTree);

    rrtTree_.clear();
    rrtTree_.resize(0);

    path.clear();
    path.resize(0);

    obstacle.clear();
    obstacle.resize(0);

}

void RRT::addNode(NodePtr qnew)
{
    // Update graph vectors
    rrtTree_.push_back(qnew);

    uintptr_t id = rrtTree_.size() - 1;
    kd_insert(kdTree, qnew->getCoord().data(), (void*) id);
}

void RRT::run(bool isWrite = true)
{
    srand((unsigned)time(NULL));
    while(state != STEP_REACHED && count < max_count)
    {
        if(rand() / double(RAND_MAX) < 0.9)
            randomStep();
        else
        {
            directStep();
            int try_count = 0;
            while(state == STEP_COLLISION && try_count++ < 10)
                bestRandomStep();
        }
        ++count;
    }
    if(state == STEP_REACHED )
    {
        printPath();
        if(path.size() < max_depth)
            ROS_INFO("Success, final tree has %d nodes!", int(path.size()));
        else
            ROS_INFO("The final depth of RRT tree exceeds the Max depth !");
        if(isWrite)
        {
            writeFile();
            printTree();
        }
    }
    else
        ROS_INFO("Fail !");
}

void RRT::randomStep()
{
    NodePtr randpoint(new Node());
    int nearID = getNearestNeighbor(randpoint);

    state = tryStep(randpoint, nearID);
}

void RRT::bestRandomStep()
{
    NodePtr randpoint(new Node());
    state = tryStep(randpoint, bestID);
}

int RRT::getBestID()
{
    return bestID;
}

bool RRT::getState()
{
    return state == STEP_REACHED;
}

/*
void RRT::randomDirectStep()
{
    int nearID = getNearestNeighbor(goal_);
    while(state == STEP_PROGRESS)
    {
        state = tryStep(goal_, nearID);
    }
}
*/

void RRT::directStep()
{
    while(state == STEP_PROGRESS)
    {
        int nearID = getNearestNeighbor(goal_);
        state = tryStep(goal_, nearID);
    }
}

void RRT::normalize(Coord &vec)
{
    //向量范数计算
    double norm = 0;
    for(int i=0; i<vec.size(); ++i)
       norm += pow(vec[i], 2);
    norm = sqrt(norm);

    for(int i=0; i<vec.size(); ++i)
        vec[i] = vec[i] / norm;
}

RRT::StepResult RRT::tryStep(const NodePtr randpoint, int nearID)
{
    NodePtr &nearpoint = rrtTree_[nearID];
    Coord diff = randpoint->getCoord() - nearpoint->getCoord();
    normalize(diff);

    // 取样点筛选
//    double temp = atan2(diff[1], diff[0]);
//    if(fabs(temp - nearpoint->getCoord()[2]) > 0.8)
//        return STEP_COLLISION;

//    double step = RANDNM(0.0, stepsize);
    Coord qnew = nearpoint->getCoord() + diff * stepsize;

    if(!checkCollisions(qnew) && !checkJointLimits(qnew))
    {
        NodePtr newpoint(new Node(qnew, nearID));
        addNode(newpoint);

//        rewireJoint(newpoint);

        double delta = deltaNode(newpoint, goal_);
        if(delta < bestDelta)
        {
            bestDelta = delta;
            bestID = rrtTree_.size()-1;
        }

        //std::cout<<bestConf<<"\t"<<limit<<std::endl;
        if(bestDelta < limit)
        {
            return STEP_REACHED;
        }
        //std::cout<<"OK---"<<std::endl;
        return STEP_PROGRESS;
    }
    else
    {
        return STEP_COLLISION;
    }
}

void RRT::rewireJoint(const NodePtr& ptr)
{
    NodePtr &parent_node = rrtTree_[ptr->parent_];
    if(parent_node->parent_ > 0)
    {
        Coord joint_pp = rrtTree_[parent_node->parent_]->getCoord();
        Coord joint_p = parent_node->getCoord();
        Coord vec_pp = joint_p - joint_pp;
        Coord vec_p = ptr->getCoord() - joint_pp;

        double vec_sum1 = 0, vec_sum2 = 0;
        for(unsigned int k=0; k<2; ++k)
        {
            vec_sum1 = pow(vec_pp[k], 2);
            vec_sum2 = pow(vec_p[k], 2);
        }
        if(vec_sum2 <= vec_sum1)
        {
//            cout <<" ***************************** " << endl;
            ptr->setParent(parent_node->parent_);
        }
    }
}

double RRT::deltaNode(const NodePtr node1, const NodePtr node2)
{
    double delta = 0;
    double weight[3] = {1.0, 1.0, 0.1};
    for(int j=0; j<node1->getCoord().size(); ++j)
        delta += weight[j]*pow(node1->getCoord()[j] - node2->getCoord()[j], 2);

    delta = sqrt(delta);
    return delta;
}

bool RRT::checkCollisions(const Coord &joints)
{
    for(unsigned int count=0; count<obstacle.size(); ++count)
    {
        if(checkSingleCollision(obstacle[count], joints))
            return true;
    }

    return false;
}

bool RRT::checkSingleCollision(const Coord& coord, const Coord& joints)
{
    double temp = robot_radius * 2;
    if((fabs(coord[0] - joints[0])<temp) && (fabs(coord[1] -joints[1])<temp))
    {
        return true;
    }
    return false;
}

bool RRT::checkJointLimits(const Coord &joints)
{
    for(int i=0; i<joints.size(); ++i)
    {
        if(joints[i]>max_dist || joints[i]<min_dist)
        {
            return true;
            //break;
		}
    }
    return false;
}

int RRT::getNearestNeighbor(const NodePtr node)
{
    struct kdres* result = kd_nearest(kdTree, node->getCoord().data());
    uintptr_t nearest = (uintptr_t)kd_res_item_data(result);
    return nearest;
}

void RRT::printPath()
{
    path.push_back(goal_->getCoord());
    NodePtr new_node = rrtTree_[bestID];
    while(new_node->parent_ > 0)
    {
        path.push_back(new_node->getCoord());
		new_node = rrtTree_[new_node->parent_];
    }
    path.push_back(init_->getCoord());
    directCut();
}

void RRT::printTree()
{
    std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream rrtTree( (filepath+"/data/rrt_tree.dat").c_str());

    size_t size_p = rrtTree_.size();
    for(size_t i=0; i<size_p; ++i)
    {
        for(size_t j=0; j<rrtTree_[0]->getCoord().size(); ++j)
        {
            rrtTree<<rrtTree_[i]->getCoord()[j]<<"\t";
        }
        rrtTree<<"\n";
    }

    rrtTree.close();

}

void RRT::writeFile()
{
    std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream rrtCoords( (filepath+"/data/coordinates.dat").c_str());

    size_t size_p = path.size();
    for(size_t i=0; i<size_p; ++i)
    {
        for(size_t j=0; j<path[0].size(); ++j)
        {
            rrtCoords<<path[size_p - i - 1][j]<<"\t";
        }
        rrtCoords<<std::endl;
    }

    rrtCoords.close();
}

void RRT::directCut()
{
    unsigned int max_part = 1;
    for(unsigned int i=0; i<path.size()-2; ++i)
    {
        max_part = max_part > i ? max_part : i+1;
        for(unsigned int j=path.size()-1; j>max_part; --j)
        {
            if(checkCollisionAndModify(i, j))
            {
                max_part = j;
                break;
            }
        }
    }
}

bool RRT::checkCollisionAndModify(unsigned int start, unsigned int end)
{
    double sample_size = 0.1;
    Coord diff = path[end] - path[start];
    unsigned int piece = sqrt(pow(diff[0],2) + pow(diff[1],2))/sample_size;

    vector<Coord> part_path;
    for(unsigned int i=1; i<=piece; ++i)
    {
        Coord temp = path[start] + diff * (1.0/piece) * i;
        part_path.push_back(temp);
        if(checkCollisions(temp))
        {
            cout << "Direct Line Collision!" << endl;
            return false;
        }
    }

    path.erase(path.begin()+start+1, path.begin()+end);
    //    cout << "start: " << start << endl;
    //    cout << path.size() << endl;
    path.insert(path.begin()+start+1, part_path.begin(), part_path.end());
    return true;
}
