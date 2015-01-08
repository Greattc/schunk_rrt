#include "schunk_rrt/mobile_rrt.h"
#include <ros/ros.h>
#include <ros/package.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

static const std::string PACKAGE = "schunk_rrt";

bool checkSingleCollision(const Coord& coord, const Coord& joints)
{
    double temp = robot_radius * 2;
    if(fabs(coord[0] - joints[0])<temp && fabs(coord[1] -joints[1])<temp)
        return true;
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_mobile_rrt");

    Coord init_vec(3);
    init_vec[0] = init_vec[1] = -1.0;
    init_vec[2] = 0.0;

    Coord goal_vec(3);
    goal_vec[0] = goal_vec[1] = 1.9;
    goal_vec[2] = 1.0;

//    std::vector<double> goal_vec(7, 0.5);

    NodePtr start(new Node(init_vec));
    NodePtr goal(new Node(goal_vec));

    ros::WallTime start_time = ros::WallTime::now();

    int success_n = 0;
//    for(int count=0; count<2000; ++count)
    for(int count=0; count<1; ++count)
    {
        RRT rrt(start, goal);

        std::string filepath = ros::package::getPath(PACKAGE);
        std::ifstream rrtCoords((filepath+"/data/obstacle_position.dat").c_str());

        while(rrtCoords)
        {
            Coord coord(2);
            rrtCoords >> coord[0] >> coord[1];
            if(!(checkSingleCollision(coord, init_vec) || checkSingleCollision(coord, goal_vec)
                 || (coord[0] == 0 && coord[1] == 0)))
            {
                rrt.setParams(coord);
            }
        }
        rrtCoords.close();

//        rrt.run(false);
        rrt.run(true);
        if(rrt.getState())
            ++success_n;
    }
    cout << success_n << endl;

    cout << (ros::WallTime::now() - start_time).toSec() << endl;

    return 0;
}
