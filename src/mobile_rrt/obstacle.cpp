#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>

class ObstacleMarker
{
public:

	ros::NodeHandle n;

	//Indicate the state of the button
	bool isTouch_axes0_;
	bool isTouch_axes1_;

	geometry_msgs::Point points_;
    geometry_msgs::Point pointsN_[2];

	visualization_msgs::Marker marker_;	

	ros::Publisher marker_pub_;
	ros::Subscriber joy_sub_;
	
	ros::Timer timer0_;
 	ros::Timer timer1_;

    bool left, down;
    double step;
 
    ObstacleMarker(): isTouch_axes0_(false), isTouch_axes1_(false)
    {
        //步长
        step = 0.1;
        marker_.header.frame_id = "/world";
//		marker_.header.stamp = ros::Time::now();
        marker_.ns = "rrt";
		marker_.action = visualization_msgs::Marker::ADD;
		marker_.pose.orientation.w = 1.0;

		marker_.id = 0;
        marker_.type = visualization_msgs::Marker::CUBE_LIST;

		//POINTS markers use x and y scale for width/height respectively
        marker_.scale.x = 0.2;
        marker_.scale.y = 0.2;
        marker_.scale.z = 0.2;
        // marker are green
        marker_.color.b = 1.0;
		marker_.color.a = 1.0;

        points_.x = points_.y = 0.5;
        points_.z = 0;
        pointsN_[0] = points_;

        points_.x = points_.y = 1.2;
        pointsN_[1] = points_;

        marker_.points.push_back(pointsN_[0]);
        marker_.points.push_back(pointsN_[1]);

        marker_pub_ = n.advertise<visualization_msgs::Marker>("obstacle_marker", 1);
        marker_pub_.publish(marker_);

        joy_sub_ = n.subscribe("joy", 10, &ObstacleMarker::joy_cb, this);
	}	
	
	void callback0(const ros::TimerEvent& event)
	{
		int sign = down ? -1 : 1;
		std::string direction = down ? "left" : "right";
		ROS_INFO("Move %s",  direction.c_str());
        for(int i=0; i<marker_.points.size();++i)
        {
            marker_.points[i].x += sign*step;
        }
        marker_pub_.publish(marker_);
	}

	void callback1(const ros::TimerEvent& event)
	{
		int sign = left ? -1 : 1;
		std::string direction = left ? "down" : "up";
		ROS_INFO("Move %s", direction.c_str());
        for(int i=0; i<marker_.points.size();++i)
        {
            marker_.points[i].y += sign*step;
        }

        marker_pub_.publish(marker_);
	}

	void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
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

        if(joy_msg->axes[0])
		{	
			down = joy_msg->axes[0] > 0 ? true : false;

			isTouch_axes0_ = true;
			timer0_ = n.createTimer(ros::Duration(0.1), &ObstacleMarker::callback0, this);
		}

        if(joy_msg->axes[1])
		{
			left = joy_msg->axes[1] > 0 ? false : true;

			isTouch_axes1_ = true;
			timer1_ = n.createTimer(ros::Duration(0.1), &ObstacleMarker::callback1, this);
		}
		
	}
};

int main( int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_marker");
	ros::NodeHandle n;
	
	ObstacleMarker plane;

	ros::Rate r(10);

	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
}
