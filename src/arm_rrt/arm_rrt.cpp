#include "schunk_rrt/arm_rrt.h"
#include <ros/ros.h>
#include <ros/package.h>
//标准头文件
#include <fstream>
#include <ostream>
#include "schunk_rrt/cubic.h"

using namespace fcl;
using std::cout;
using std::endl;
using std::vector;

typedef boost::shared_ptr<Cylinder> CylinderPtr;
typedef boost::shared_ptr<CollisionObject> CollisionPtr;

static const std::string PACKAGE = "schunk_rrt";

Joint operator-(const Joint &v1, const Joint &v2)
{
    Joint result(v1);
    for (int i=0; i < v1.size(); ++i)
    {
        result[i] = v1[i] - v2[i];
    }
    return result;
}

Joint operator+(const Joint &v1, const Joint &v2)
{
    Joint result(v1);
    for (int i=0; i < v1.size(); ++i)
    {
        result[i] = v1[i] + v2[i];
    }
    return result;
}

Joint operator*(const Joint &v1, double scale)
{
    Joint result(v1);
    for (int i=0; i < v1.size(); ++i)
    {
        result[i] = v1[i] * scale;
    }
    return result;
}

std::ostream& operator<<(std::ostream& os, const Joint &vec)
{
    for(int i=0; i<vec.size(); ++i)
        os << vec[i] << "\n";

    return os;
}

bool vectorEqual(const Joint &a, const Joint &b)
{
    assert(a.size() == b.size() );
    vector<double> delta = a-b;
    double sum=0;
    for(int n=0; n<a.size(); ++n)
        sum += pow(delta[n], 2);
    if(sqrt(sum) < 1e-5)
        return true;
    return false;
}

Point forwardKinematics(const Joint& joint_)
{
    Eigen::MatrixXd pose(4,4);
    Eigen::MatrixXd A1(4,4), A2(4,4), A3(4,4), A4(4,4), A5(4,4), A6(4,4), A7(4,4);
    A1<<cos(joint_[0]),0,-sin(joint_[0]),0,sin(joint_[0]),0,
            cos(joint_[0]),0,0,-1,0,0.3,0,0,0,1;
    A2<<cos(joint_[1]),0,sin(joint_[1]),0,sin(joint_[1]),0,
            -cos(joint_[1]),0,0,1,0,0,0,0,0,1;
    A3<<cos(joint_[2]),0,-sin(joint_[2]),0,sin(joint_[2]),0,
            cos(joint_[2]),0,0,-1,0,0.328,0,0,0,1;
    A4<<cos(joint_[3]),0,sin(joint_[3]),0,sin(joint_[3]),0,
            -cos(joint_[3]),0,0,1,0,0,0,0,0,1;
    A5<<cos(joint_[4]),0,-sin(joint_[4]),0,sin(joint_[4]),0,
            cos(joint_[4]),0,0,-1,0,0.276,0,0,0,1;
    A6<<cos(joint_[5]),0,sin(joint_[5]),0,sin(joint_[5]),0,
            -cos(joint_[5]),0,0,1,0,0,0,0,0,1;
    A7<<cos(joint_[6]),-sin(joint_[6]),0,0,sin(joint_[6]),
            cos(joint_[6]),0,0,0,0,1,0.1785,0,0,0,1;
    pose = A1*A2*A3*A4*A5*A6*A7;

    Point point;
    for(unsigned int i=0; i<3; ++i)
        point.push_back(pose(i,3));

    return point;
}

Node::Node()
{
    joint_.push_back((rand() % 628 -314) / 100.0);
    joint_.push_back((rand() % 400 -200) / 100.0);
    joint_.push_back((rand() % 628 -314) / 100.0);
    joint_.push_back((rand() % 400 -200) / 100.0);
    joint_.push_back((rand() % 628 -314) / 100.0);
    joint_.push_back((rand() % 400 -200) / 100.0);
    joint_.push_back((rand() % 628 -314) / 100.0);

    parent_ = 0;
}

Node::Node(const Joint &vec)
{
    joint_ = vec;
    parent_ = -1;
}

Node::Node(const Joint &vec, int parent)
{
    joint_ = vec;
    parent_ = parent;
}

Joint Node::getJoint() const
{
    return joint_;
}

Point Node::getPoint()
{
    if(point_.empty())
        point_ = forwardKinematics(joint_);

    return point_;
}

int Node::getParent() const
{
    return parent_;
}

void Node::setParent(int parent)
{
    parent_ = parent;
}

RRT::RRT(const NodePtr init, const NodePtr goal)
{
    initialize(init, goal);
}

RRT::~RRT()
{
    cleanup();
}

void RRT::initialize(const NodePtr init, const NodePtr goal)
{
    rrtTree_.push_back(init);
    bestDelta = 100000;
    bestID = 0;
    count = 0;

    init_ = init;
    goal_ = goal;
    kdTree = kd_create(7);
    addNode(init);

    state = STEP_PROGRESS;

    min_.push_back(-M_PI);
    min_.push_back(-2.0);
    min_.push_back(-M_PI);
    min_.push_back(-2.0);
    min_.push_back(-M_PI);
    min_.push_back(-2.0);
    min_.push_back(-M_PI);

    max_.push_back(M_PI);
    max_.push_back(2.0);
    max_.push_back(M_PI);
    max_.push_back(2.0);
    max_.push_back(M_PI);
    max_.push_back(2.0);
    max_.push_back(M_PI);

    //初始碰撞次数为0
    collision_n = 0;
    collision_op = 0;
    collision_joint.resize(7);
    for(int i=0; i<7; ++i)
        collision_joint[i] = 0;
}

boost::shared_ptr<ShapeBase> RRT::getObstacle(int index) const 
{
    return collisionGroup[index];
}

/*
void RRT::setObstacle(const vector<Joint>& obstacle_)
{
    obstacle = obstacle_;
}
*/

NodePtr RRT::getGoal() const
{
    return goal_;
}

void RRT::addObstacle(const boost::shared_ptr<Sphere> object, const Transform3f& tf)
//void RRT::addObstacle(const boost::shared_ptr<Box> object, const Transform3f& tf)
{
    collisionGroup.push_back(object);
    collisionTFGroup.push_back(tf);
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

    traj.clear();
    traj.resize(0);
    
    obstacle.clear();
    obstacle.resize(0);
}

void RRT::addNode(const NodePtr &qnew)
{
    // Update graph vectors
    rrtTree_.push_back(qnew);

    uintptr_t id = rrtTree_.size() - 1;
    kd_insert(kdTree, qnew->getJoint().data(), (void*) id);
}

void RRT::run(bool isWrite = true)
{
    srand((unsigned)time(NULL));
    while(state != STEP_REACHED && count < max_count)
    {
        if(rand() / double(RAND_MAX) < 0.1)
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
        printTrajectory();
        printModifiedPath();

        if(traj.size() < max_depth)
            ROS_INFO("Success, final tree has %d nodes!", int(traj.size()));
        else
            ROS_INFO("The final depth of RRT tree exceeds the Max depth !");
        if(isWrite)
        {
            writeFile();
            // printTree();
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

void RRT::normalize(Joint &vec)
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
    Joint diff = randpoint->getJoint() - nearpoint->getJoint();
    normalize(diff);

    // 取样点筛选
//    double temp = atan2(diff[1], diff[0]);
//    if(fabs(temp - nearpoint->getJoint()[2]) > 0.8)
//        return STEP_COLLISION;

//    double step = RANDNM(0.0, stepsize);
    Joint qnew = nearpoint->getJoint() + diff * stepsize;

    if(!checkCollisions(qnew) && !checkJointLimits(qnew))
    {
        NodePtr newpoint(new Node(qnew, nearID));
        addNode(newpoint);

        //rewireJoint(newpoint);

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
    NodePtr &parent_node = rrtTree_[ptr->getParent()];
    if(parent_node->getParent() > 0)
    {
        Joint joint_pp = rrtTree_[parent_node->getParent()]->getJoint();
        Joint joint_p = parent_node->getJoint();
        Joint vec_pp = joint_p - joint_pp;
        Joint vec_p = ptr->getJoint() - joint_pp;

        double vec_sum1 = 0, vec_sum2 = 0;
        for(unsigned int k=0; k<2; ++k)
        {
            vec_sum1 = pow(vec_pp[k], 2);
            vec_sum2 = pow(vec_p[k], 2);
        }
        if(vec_sum2 <= vec_sum1)
        {
//            cout <<" ***************************** " << endl;
            ptr->setParent(parent_node->getParent());
        }
    }
}

double RRT::deltaNode(const NodePtr node1, const NodePtr node2)
{
    double delta = 0;

    for(int j=0; j<node1->getJoint().size(); ++j)
        delta += pow(node1->getJoint()[j] - node2->getJoint()[j], 2);

    return sqrt(delta);
}

bool RRT::checkCollisions(const Joint &joints)
{
    ++collision_n;
    static Eigen::VectorXd radius(N_DIM);
    static Eigen::VectorXd length(N_DIM);
    radius << 0.07, 0.07, 0.06, 0.06, 0.05, 0.05, 0.04;
    length << 0.3, 0.17, 0.328, 0.16, 0.276, 0.14, 0.1785;

    Eigen::MatrixXd A1(4,4),A2(4,4),A3(4,4),A4(4,4),A5(4,4),A6(4,4),A7(4,4),A(4,4);
    A1<<cos(joints[0]),0,-sin(joints[0]),0,sin(joints[0]),0,
    cos(joints[0]),0,0,-1,0,length(0),0,0,0,1;
    A2<<cos(joints[1]),0,sin(joints[1]),0,sin(joints[1]),0,
    -cos(joints[1]),0,0,1,0,0,0,0,0,1;
    A3<<cos(joints[2]),0,-sin(joints[2]),0,sin(joints[2]),0,
    cos(joints[2]),0,0,-1,0,length(2),0,0,0,1;
    A4<<cos(joints[3]),0,sin(joints[3]),0,sin(joints[3]),0,
    -cos(joints[3]),0,0,1,0,0,0,0,0,1;
    A5<<cos(joints[4]),0,-sin(joints[4]),0,sin(joints[4]),0,
    cos(joints[4]),0,0,-1,0,length(4),0,0,0,1;
    A6<<cos(joints[5]),0,sin(joints[5]),0,sin(joints[5]),0,
    -cos(joints[5]),0,0,1,0,0,0,0,0,1;
    A7<<cos(joints[6]),-sin(joints[6]),0,0,sin(joints[6]),
    cos(joints[6]),0,0,0,0,1,length(6),0,0,0,1;

    //构造7个用来检测碰撞的圆柱体tf
    vector<Transform3f> tfGroup;
    vector<CylinderPtr> cylinderGroup;

    for(unsigned int i=0; i<N_DIM; ++i)
        cylinderGroup.push_back(CylinderPtr(new Cylinder(radius(i),length(i))));

    A = A1;
    Vec3f shoulder(A(0,3),A(1,3),A(2,3));
    Vec3f z_1(A(0,2),A(1,2),A(2,2));
    //每个圆柱体的旋转矩阵
    fcl::Matrix3f z1;
    z1.setIdentity();
    tfGroup.push_back(Transform3f(z1, shoulder * 0.5));

    fcl::Matrix3f z2(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));
    tfGroup.push_back(Transform3f(z2, shoulder));

    A = A*A2;
    fcl::Matrix3f z3(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));

    A = A*A3;
    Vec3f elbow(A(0,3),A(1,3),A(2,3));
    fcl::Matrix3f z4(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));
    tfGroup.push_back(Transform3f(z3, (elbow + shoulder) * 0.5));
    tfGroup.push_back(Transform3f(z4, elbow));

    A = A*A4;
    fcl::Matrix3f z5(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));

    A = A*A5;
    Vec3f z_5(A(0,2),A(1,2),A(2,2));
    Vec3f wrist(A(0,3),A(1,3),A(2,3));
    fcl::Matrix3f z6(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));
    tfGroup.push_back(Transform3f(z5,  (wrist + elbow) * 0.5));
    tfGroup.push_back(Transform3f(z6, wrist));

    A = A*A6;
    fcl::Matrix3f z7(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2));

    A = A*A7;
    Vec3f hand(A(0,3),A(1,3),A(2,3));
    tfGroup.push_back(Transform3f(z7, (wrist + hand) * 0.5));

    //      GJKSolver_indep solver;
    GJKSolver_libccd solver;

    for(unsigned int k=0; k<collisionGroup.size(); ++k)
    {
        for(int i = N_DIM -1; i >= 0; --i){
            Vec3f contact_points;
            FCL_REAL penetration_depth;
            Vec3f normal;

            bool res = solver.shapeIntersect(*cylinderGroup[i], tfGroup[i], *(collisionGroup[k]), collisionTFGroup[k], &contact_points, &penetration_depth, &normal);
//        cout << "contact points: " << contact_points << endl;
//        cout << "pen depth: " << penetration_depth << endl;
//        cout << "normal: " << normal << endl;
            if(res)
            {
                ++collision_joint[i];
//                ROS_INFO("Collision in Joint : %d", i+1);
                return true;
            }
//        else
//            ROS_INFO("No Collision!");
        }
    }
    return false;
}

bool RRT::checkJointLimits(const Joint &joints)
{
    for(int i=0; i<N_DIM; ++i)
    {
        if(joints[i]>max_[i] || joints[i]<min_[i] )
        {
            return true;
            //break;
		}
    }
    return false;
}

int RRT::getNearestNeighbor(const NodePtr node)
{
    struct kdres* result = kd_nearest(kdTree, node->getJoint().data());
    uintptr_t nearest = (uintptr_t)kd_res_item_data(result);
    return nearest;
}

void RRT::printPath()
{
    path.push_back(goal_->getPoint());
    NodePtr new_node = rrtTree_[bestID];
    while(new_node->getParent() > 0)
    {
        path.push_back(new_node->getPoint());
        new_node = rrtTree_[new_node->getParent()];
    }
//    path.push_back(init_->getPoint());
}

void RRT::printModifiedPath()
{
    unsigned int size_j = traj.size();
    for(unsigned int i=0; i< size_j; ++i)
    {
        mpath.push_back(forwardKinematics(traj[i]));
    }
}

void RRT::writeSmoothPath()
{
    int size_s = straj.size();
    for(int i=0; i< size_s; ++i)
    {
        spath.push_back(forwardKinematics(straj[i]));
    }
    std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream rrtSPath( (filepath+"/data/smooth_points.dat").c_str());

    int size_p = spath.size();
    for(int i=0; i<size_p; ++i)
    {
        for(int j=0; j<spath[0].size(); ++j)
        {
            rrtSPath<<spath[size_p - i - 1][j]<<"\t";
        }
        rrtSPath<<"\n";
    }
    rrtSPath.close();
}

void RRT::printTrajectory()
{
    traj.push_back(goal_->getJoint());
    NodePtr new_node = rrtTree_[bestID];
    while(new_node->getParent() > 0)
    {
        traj.push_back(new_node->getJoint());
        new_node = rrtTree_[new_node->getParent()];
    }
    traj.push_back(init_->getJoint());
    directCut();
}

void RRT::printTree()
{
    std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream rrtTree( (filepath+"/data/rrt_tree.dat").c_str());

    size_t size_p = rrtTree_.size();
    for(size_t i=0; i<size_p; ++i)
    {
        for(size_t j=0; j<rrtTree_[0]->getJoint().size(); ++j)
        {
            rrtTree<<rrtTree_[i]->getJoint()[j]<<"\t";
        }
        rrtTree<<"\n";
    }

    rrtTree.close();
}

void RRT::writeFile()
{
    printPath();

    std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream rrtJoints( (filepath+"/data/joints.dat").c_str());
    std::ofstream rrtPoints( (filepath+"/data/points.dat").c_str());
    std::ofstream rrtMPoints( (filepath+"/data/mpoints.dat").c_str());

    size_t size_p = traj.size();
    for(size_t i=0; i<size_p; ++i)
    {
        for(size_t j=0; j<traj[0].size(); ++j)
        {
            rrtJoints<<traj[size_p - i - 1][j]<<"\t";
        }
        rrtJoints<<"\n";
    }

    size_t size_j = path.size();
    for(size_t i=0; i<size_j; ++i)
    {
        for(size_t j=0; j<path[0].size(); ++j)
        {
            rrtPoints<<path[size_j - i - 1][j]<<"\t";
        }
        rrtPoints<<"\n";
    }

    size_t size_m = mpath.size();
    for(size_t i=0; i<size_m; ++i)
    {
        for(size_t j=0; j<mpath[0].size(); ++j)
        {
            rrtMPoints<<mpath[size_m - i - 1][j]<<"\t";
        }
        rrtMPoints<<"\n";
    }

    rrtJoints.close();
    rrtPoints.close();
    rrtMPoints.close();
}

void RRT::writeSmoothTraj()
{
    std::string filepath = ros::package::getPath(PACKAGE);
    std::ofstream smoothFile( (filepath+"/data/smooth_joints.dat").c_str());

    size_t size_p = straj.size();
    for(size_t i=0; i<size_p; ++i)
    {
        for(size_t j=0; j<straj[0].size(); ++j)
        {
            smoothFile <<straj[size_p - i - 1][j]<<"\t";
        }
        smoothFile<<"\n";
    }
}

void RRT::smooth(const vector<double> &init_vec, const vector<double> &end_vec, bool need, double s_time)
{
    int gap = 3;
    double time_node = 0.04;

    //RRT算法两点间的运行时间
    vector<double> v_start = (traj[gap+1]-traj[gap]) * (1/time_node) ;

//    smoothPart(gap,0, v_start, end_vec, 0.3);
    smoothPart(0,gap,end_vec, v_start, 0.3);

    vector<double> vec_gap = traj[gap+1]-traj[gap];
    int break_point = gap;

    //改善动态ＲＲＴ算法的性能突破点
    int smooth_p = need ? (traj.size()-4) : (traj.size());
    for(; break_point<smooth_p; ++break_point)
    {
        vector<double> vec_test = traj[break_point+1]-traj[break_point];
        if(vectorEqual(vec_gap, vec_test))
        {
            nodeSmooth(break_point, time_node);
        }
        else
        {
            cout << "Knot point" << endl;
            //去掉最后四个点
            straj.erase(straj.end()-5, straj.end());

            vector<double> next_gap = traj[break_point+2]-traj[break_point+1];
            vector<double> next_vec_gap = next_gap*(1/time_node);
            vector<double> last_vec_gap = vec_gap*(1/time_node);
            smoothPart(break_point-1,break_point+1,last_vec_gap, next_vec_gap, 0.2);
            vec_gap = next_gap;
//            ++break_point;
        }

    }

//    straj.erase(straj.begin(), straj.begin()+5);

//    straj.erase(straj.end()-1, straj.end());

    //    smoothPart(break_point,break_point+1,last_vec_gap, next_vec_gap, 0.2);

//    nodeSmooth(gap+1, time_node);
//    nodeSmooth(gap, time_node);



//    int p = traj.size();
//    v_start = (traj[p-2]-traj[p-1])*10;
//    smoothPart(p-1, p-2, init_vec, v_start);

    straj.erase(straj.end()-1, straj.end());
    int p = traj.size();
    v_start = (traj[p-1-gap]-traj[p-2-gap]) * (1/time_node) ;
    if(need)
        smoothPart(p-1-gap,p-1,v_start, init_vec, s_time);
    else
    {
    }

    writeSmoothTraj();
}

void RRT::nodeSmooth(int start, double time)
{
//    cout << traj[start+1] << endl;
//    cout << traj[start] << endl;
    int piece = (int)(time*Herz);

    vector<double> v_start = (traj[start+1]-traj[start]) * (1.0/piece);

    for(int i=0; i<piece; ++i)
    {
        vector<double> temp = traj[start] + v_start*(i+1);

        straj.push_back(temp);
    }
}

void RRT::smoothPart(int start, int end, const vector<double> &start_vec, const vector<double> &end_vec, double time)
{
    vector<CubicSpline> cubic_joint;
    for(int i=0; i<7; ++i)
    {
        CubicSpline cubic_temp;
        cubic_temp = cubicSolve(traj[start][i], traj[end][i], start_vec[i], end_vec[i], time);
        cubic_joint.push_back(cubic_temp);
    }

    for(int i=0; i<=time*Herz; ++i)
    {
        vector<double> temp;
        //cout<<i/Herz<<endl;
        for(int j=0; j<7; ++j)
        {
            temp.push_back( cubicValue(cubic_joint[j], i/Herz) );
        }
        straj.push_back(temp);
    }
}

void RRT::directCut()
{
    unsigned int max_part = 1;
    for(unsigned int i=0; i<traj.size()-2; ++i)
    {
        max_part = max_part > i ? max_part : i+1;
        for(unsigned int j=traj.size()-1; j>max_part; --j)
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
    static double ss = 0.02;
    Joint diff = traj[end] - traj[start];
    unsigned int piece = sqrt(pow(diff[0],2) + pow(diff[1],2))/ss;

    vector<Joint> part_traj;
    for(unsigned int i=1; i<=piece; ++i)
    {
        //统计优化时的碰撞检测次数
        ++collision_op;
        Joint temp = traj[start] + diff * (1.0/piece) * i;
        part_traj.push_back(temp);
        if(checkCollisions(temp))
        {
//            cout << "Direct Line Collision!" << endl;
            return false;
        }
    }

    traj.erase(traj.begin()+start+1, traj.begin()+end);
    //    cout << "start: " << start << endl;
    //    cout << traj.size() << endl;
    traj.insert(traj.begin()+start+1, part_traj.begin(), part_traj.end());
//    traj.erase(traj.end(), traj.end()+1);
    return true;
}
