#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>
#include <schunk_rrt/kdtree.h>

using std::vector;

#define RANDNM(N,M) N + ((M-N) * (rand() / ((double)RAND_MAX + 1))) // random # between N&M

static int max_count = 10000;
static double stepsize = 0.1;
//static double stepsize = 0.1;
static double limit = 0.1;
static int max_depth = 80;
static int N_DIM = 3;
static double robot_radius = 0.1;
static double min_dist = -2.0;
static double max_dist = 2.0;

//typedef Eigen::Matrix<double,3,1> Point;
typedef vector<double> Coord;//Coordinate

class Node
{
public:
    Node();
	~Node(){};

    Node(const Coord &vec);
    Node(const Coord &vec, int parent);

    Coord getCoord();
    void setParent(int parent);

    Coord pos_;
    int parent_;//父节点
};

typedef boost::shared_ptr<Node> NodePtr;
typedef vector<Coord> trajectory;

class RRT
{
public:

    typedef enum
    {
        STEP_COLLISION, /**< Collided with obstacle. No added */
        STEP_REACHED,    /**< The configuration that we grow to is less than stepSize away from node we grow from. No node added */
        STEP_PROGRESS    /**< One node added */
    }  StepResult;

    RRT(NodePtr init, NodePtr goal);
    ~RRT();
    void run(bool isWrite);
    void addNode(NodePtr qnew);
    double deltaNode(const NodePtr node1, const NodePtr node2);

    void setParams(Coord vec);
    void clearObstacle();
    bool checkCollisions(const Coord &joints);
    NodePtr getGoal() const;
    int getBestID();
    bool getState();
    vector<Coord> getObstacle();
    void setObstacle(const vector<Coord>& obstacle_);
    double getRadius();
    vector<NodePtr> rrtTree_;
    trajectory path;
    //关节角度限制
    Coord min_, max_;

private:

    void initialize(NodePtr init, NodePtr goal);
    StepResult tryStep(const NodePtr randpoint, int nearID);
    void randomStep();
    void directStep();
    void bestRandomStep();
    void cleanup();

    void rewireJoint(const NodePtr& ptr);
    void normalize(Coord &vec);
    void printPath();
    void printTree();
    int getNearestNeighbor(const NodePtr node);
    bool checkJointLimits(const Coord &joints);
    bool checkSingleCollision(const Coord& coord, const Coord& joints);
    void writeFile();
    void directCut();
    bool checkCollisionAndModify(unsigned int start, unsigned int end);

    NodePtr init_;
    NodePtr goal_;
    int count;
    double bestDelta;
    int bestID;
    StepResult state;
    struct kdtree *kdTree;
    int prevSize_;
    vector<Coord> obstacle;
};

typedef boost::shared_ptr<RRT> RRTPtr;
