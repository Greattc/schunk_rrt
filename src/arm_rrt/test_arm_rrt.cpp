#include "schunk_rrt/arm_rrt.h"
#include "ros/ros.h"

#include "schunk_kinematics/arm_kinematics.h"

using namespace std;
using namespace fcl;
using Eigen::VectorXd;
using Eigen::MatrixXd;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_rrt");

    vector<double> init_vec(7, 0.0);

    VectorXd IK(7), linkLen(7), IK0(7);
    vector<double> goal_vec(7);

    MatrixXd Goal(4,4);
    Goal<<0.0, 0.0, 1.0, 0.4,\
          1.0, 0.0, 0.0, 0.0,\
          0.0, 1.0, 0.0, 0.3,\
          0.0, 0.0, 0.0, 1.0;

    linkLen<<0.3,0.0,0.328,0.0,0.276,0.0,0.1785;  //Grasp Link

    IK0 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
//    IK0 << 0.8407, 1.2096, 0.0, 0.63, 0.0, 0.0, 0.0;
//    inverse_kinematics(IK, linkLen, Goal, false, 0);
//    minimum_energy(IK, IK0);

    Goal<< 0.0, -1.0, 0.0,-0.4,\
           0.0,  0.0, 1.0, 0.6,\
          -1.0,  0.0, 0.0, 0.3,\
           0.0,  0.0, 0.0, 1.0;

//    IK0 << 0.8407, 1.2096, 0.0, 0.635, 0.0, 0.0, 0.0;
//    IK0 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    IK0 << 0.83801417976, 0.8428328, 0.5292391493999999, 0.9932776000000003, -1.45556424048, 0.27745, 0.0;

//    cout << forward_kinematics(IK0, linkLen) << endl;

    IK << 2.168, 1.2096, 0.0, 0.6350, 0.0, -0.2268, 0.0;
    inverse_kinematics(IK, linkLen, Goal, false, 0);
    minimum_energy(IK, IK0);

    for(int i=0; i<7; ++i)
    {
        init_vec[i] = IK0(i);
        goal_vec[i] = IK(i);
    }

    //始末位置
    NodePtr start(new Node(init_vec));
    NodePtr goal(new Node(goal_vec));

/*
    //障碍物
    boost::shared_ptr<Sphere> sphere(new Sphere(0.2));
    Transform3f tf1;
    tf1.setIdentity();
    tf1.setTranslation(Vec3f(0.5, 0.1, 0.3));
    //障碍物
*/

    //第一个障碍物，方体。
    boost::shared_ptr<Box> box(new Box(0.2,0.2,0.05));
    Transform3f tf1;
    tf1.setIdentity();
    tf1.setTranslation(Vec3f(0.6,-0.2, 0.7));

    //第二个障碍物，方体。
    boost::shared_ptr<Box> box2(new Box(0.2,0.2,0.05));
    Transform3f tf2;
    tf2.setIdentity();
    tf2.setTranslation(Vec3f(0.4,-0.3, 0.9));

    //第三个障碍物，球体。
    boost::shared_ptr<Sphere> sphere(new Sphere(0.2));
    Transform3f tf3;
    tf3.setIdentity();
    tf3.setTranslation(Vec3f(0.0, 0.6, 0.2));

    int success_n = 0;
    ros::WallTime start_time = ros::WallTime::now();
    vector<int> sum_of_joint_collision(7,0);

    int collision_check_total = 0;
    int depth_total = 0;

//    for(int count=0; count < 1000; ++count)
    for(int count=0; count < 1; ++count)
    {
        RRT rrt(start, goal);

        rrt.addObstacle(sphere, tf3);
//        rrt.addObstacle(box, tf1);
//        rrt.addObstacle(box2, tf2);

//        rrt.run(false);
        rrt.run(true);
        if(rrt.getState())
        {
            vector<double> init_vec(7, 0.0);
            vector<double> end_vec(7, 0.0);
            rrt.smooth(init_vec, end_vec);
            rrt.writeSmoothPath();

            ++success_n;
        }

//        for(int i=0; i<7; ++i)
//            sum_of_joint_collision[i] += rrt.collision_joint[i];

        collision_check_total += rrt.collision_n;
        depth_total += rrt.traj.size();
//        cout << rrt.collision_n << "\t" << rrt.collision_op << endl;
    }
//    cout << success_n << endl;

//    for(int i=0; i<7; ++i)
//        cout << sum_of_joint_collision[i] << endl;

//    cout << collision_check_total << endl;
//    cout << depth_total << endl;
    cout << "Total RRT running time: " << (ros::WallTime::now()-start_time).toSec() << "s\n";

    return 0;
}
