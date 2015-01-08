#include "schunk_rrt/dynamic_obstacle.h"

ObstacleMarker::ObstacleMarker(): isTouch_axes0_(false), isTouch_axes1_(false), isTouch_axes2_(false), isTouch_axes3_(false)
{
    //步长
    step = 0.1;
    marker_.header.frame_id = "/world";
    //		marker_.header.stamp = ros::Time::now();
    marker_.ns = "rrt";
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.orientation.w = 1.0;

    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::SPHERE;

    //POINTS markers use x and y scale for width/height respectively
    marker_.scale.x = 0.2;
    marker_.scale.y = 0.2;
    marker_.scale.z = 0.2;
    // marker are green
    marker_.color.r = 1.0;
    marker_.color.a = 1.0;

    marker_.pose.position.x = 0.0;
    marker_.pose.position.y = 0.0;
    marker_.pose.position.z = 0.0;

    marker_pub_ = n.advertise<visualization_msgs::Marker>("dynamic_obstacle", 1);

    marker_pub_.publish(marker_);

    joy_sub_ = n.subscribe("joy", 10, &ObstacleMarker::joy_cb, this);
}	

void ObstacleMarker::callback0(const ros::TimerEvent& event)
{
    int sign = down ? -1 : 1;
    std::string direction = down ? "left" : "right";
    ROS_INFO("Move %s",  direction.c_str());
    marker_.pose.position.x += sign*step;
    marker_pub_.publish(marker_);
}

void ObstacleMarker::callback1(const ros::TimerEvent& event)
{
    int sign = left ? -1 : 1;
    std::string direction = left ? "backward" : "forward";
    ROS_INFO("Move %s", direction.c_str());
    marker_.pose.position.y += sign*step;
    marker_pub_.publish(marker_);
}

void ObstacleMarker::callback2(const ros::TimerEvent& event)
{
    std::string direction = "up";
    ROS_INFO("Move %s", direction.c_str());
    marker_.pose.position.z += step;
    marker_pub_.publish(marker_);
}

void ObstacleMarker::callback3(const ros::TimerEvent& event)
{
    std::string direction = "down";
    ROS_INFO("Move %s", direction.c_str());
    marker_.pose.position.z -= step;
    marker_pub_.publish(marker_);
}

void ObstacleMarker::joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    //Justify the state of the last state!
    if(isTouch_axes0_ && !joy_msg->axes[0] && timer0_.isValid())
    {
        timer0_.stop();
    }
    //Justify the state of the last state!
    if(isTouch_axes1_ && !joy_msg->axes[1] && timer1_.isValid())
    {
        timer1_.stop();
    }
    if(isTouch_axes2_ && !joy_msg->buttons[4] && timer2_.isValid())
    {
        timer2_.stop();
    }
    if(isTouch_axes3_ && !joy_msg->buttons[5] && timer3_.isValid())
    {
        timer3_.stop();
    }

    if(joy_msg->axes[0])
    {
        down = joy_msg->axes[0] > 0 ? true : false;

        isTouch_axes0_ = true;
        timer0_ = n.createTimer(ros::Duration(0.2), &ObstacleMarker::callback0, this);
    }

    if(joy_msg->axes[1])
    {
        left = joy_msg->axes[1] > 0 ? false : true;

        isTouch_axes1_ = true;
        timer1_ = n.createTimer(ros::Duration(0.2), &ObstacleMarker::callback1, this);
    }

    if(joy_msg->buttons[4])
    {
        isTouch_axes2_ = true;
        timer2_ = n.createTimer(ros::Duration(0.2), &ObstacleMarker::callback2, this);
    }
    if(joy_msg->buttons[5])
    {
        isTouch_axes3_ = true;
        timer3_ = n.createTimer(ros::Duration(0.2), &ObstacleMarker::callback3, this);
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "obstacle_marker");

	ROS_INFO("Dynamic Obstatcle Starting... ");
    ObstacleMarker obstacle;

    ros::spin();
}

