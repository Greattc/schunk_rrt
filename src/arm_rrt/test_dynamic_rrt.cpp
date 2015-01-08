#include "schunk_rrt/dynamic_rrt.h"
#include <boost/shared_ptr.hpp>
#include <ros/callback_queue.h>

#include "schunk_kinematics/arm_kinematics.h"

using namespace fcl;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamicRRT");

    std::vector<double> init_vec(7, 0.0);
    std::vector<double> goal_vec(7, 1.0);
//    std::vector<double> goal_vec(7, 0.5);

    VectorXd IK(7), linkLen(7), IK0(7);

    MatrixXd Goal(4,4);
    Goal<< 0.0, -1.0, 0.0,-0.4,\
           0.0,  0.0, 1.0, 0.6,\
          -1.0,  0.0, 0.0, 0.3,\
           0.0,  0.0, 0.0, 1.0;

    linkLen<<0.3,0.0,0.328,0.0,0.276,0.0,0.1785;  //Grasp Link
    IK0 << 0.8407, 1.2096, 0.0, 0.635, 0.0, 0.0, 0.0;

    IK << 2.168, 1.2096, 0.0, 0.6350, 0.0, -0.2268, 0.0;
    inverse_kinematics(IK, linkLen, Goal, false, 0);
    minimum_energy(IK, IK0);

//    cout << forward_kinematics(IK, linkLen) << endl;
//    cout << forward_kinematics(IK0, linkLen) << endl;

    for(int i=0; i<7; ++i)
    {
        init_vec[i] = IK0(i);
        goal_vec[i] = IK(i);
    }

    NodePtr start(new Node(init_vec));
    NodePtr goal(new Node(goal_vec));

    ros::NodeHandle nh1;

    //second nodehandle and service queue for working in second thread
    ros::NodeHandle nh2(nh1);
    ros::CallbackQueue service_queue(true);
    nh2.setCallbackQueue(&service_queue);


    DynamicRRT rrt(start, goal, nh1, nh2);
    boost::shared_ptr<Sphere> box(new Sphere(0.1));
    Transform3f tf1;
    tf1.setIdentity();
    tf1.setTranslation(Vec3f(0.0, 1.0, 0.5));
    rrt.rrt->addObstacle(box, tf1);

    ros::Rate r(2);

    ros::AsyncSpinner spinner2(1,&service_queue);
    spinner2.start();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();

    /*
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    */

    return 0;
}
