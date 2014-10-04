#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pointPublisher.h"

// Publishes a list of points to the cmd_pos topic 
void publishPoints(std::list<tp1::ccoord> points){
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	ros::Publisher cmdPosPub = n.advertise<tp1::ccoord>("cmd_pos", 10); 
	for(auto i = points.begin(), iend = points.end(); i != iend && ros::ok(); i++){
		cmdPosPub.publish(*i);
		loop_rate.sleep();
	}
}
