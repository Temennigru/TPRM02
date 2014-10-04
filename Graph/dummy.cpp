#include "ros/ros.h"
#include "../Robot/pointPublisher.h"

int main(int argc, char ** argv){
	ros::init(argc, argv, "path_designator");	
	std::list<tp1::ccoord> points;
	tp1::ccoord A; A.x = 0; A.y = 0; A.theta = 0;
	tp1::ccoord B; B.x = 0; B.y = 5; B.theta = 0;
	tp1::ccoord C; C.x = 5; C.y = 5; C.theta = 0;
	tp1::ccoord D; D.x = 5; D.y = 0; D.theta = 0;
	tp1::ccoord E; E.x = 0; E.y = 0; E.theta = 0;
	points.push_back(A);
	points.push_back(B);
	points.push_back(C);
	points.push_back(D);
	points.push_back(E);
	publishPoints(points);
}
