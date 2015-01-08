#include "schunk_rrt/dynamic_mobile_rrt.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_dynamic_rrt");

    Coord init_vec(3);
    init_vec[0] = init_vec[1] = -1.0;
    init_vec[2] = 0.0;

    Coord goal_vec(3);
    goal_vec[0] = goal_vec[1] = 1.9;
    goal_vec[2] = 1.0;

//    std::vector<double> goal_vec(7, 0.5);

    NodePtr start(new Node(init_vec));
    NodePtr goal(new Node(goal_vec));

    DynamicRRT rrt(start, goal);

    ros::Rate loop_rate(10.0);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
