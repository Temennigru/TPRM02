#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include "../Graph/CellGrid.h"
#include <list>
#include <float.h>

// Position of the robot
volatile double originX;
volatile double originY;
volatile double px;
volatile double py;
volatile double theta;
void odomCallback(const nav_msgs::Odometry &msg){
	px = msg.pose.pose.position.x + originX;
	py = msg.pose.pose.position.y + originY;
	theta = asin(msg.pose.pose.orientation.z)*2;
}

// Laser callback (grid update)
probabilisticCellGrid_t * cellGrid;
const float rayInc = 0.2;
bool isStopped = false;
void laserCallback(const sensor_msgs::LaserScan &msg){	
	if(!isStopped) return;

	// Iterate over each ray, and increment hit/miss grid allong its path 	
	size_t idx = 0;
	for(float phi = msg.angle_min + theta; phi <= msg.angle_max + theta; phi += msg.angle_increment){ 

		float laserDist = msg.ranges[idx];
		if(laserDist <= msg.range_min){
			idx++;
			continue;		
		}	

		// Compute the target x and y
		float dstX, dstY;
		if(laserDist >= msg.range_max) dstX = dstY = FLT_MAX;
		else{
			dstX = px + msg.ranges[idx]*cos(phi);
			dstY = py + msg.ranges[idx]*sin(phi);
		}

		/* Runs through all points along the ray; 
		   Points are incremented roughly proportionally to the size of the intersect between the ray and the cell */
		for(float r = 0; r < laserDist; r += rayInc){
			float x = px + r*cos(phi);
			float y = py + r*sin(phi);
			bool occupied = pow(x - dstX, 2) + pow(y - dstY, 2) < 0.05*0.05;
			//sprintf("theta = %f ; phi %f ; r = %f ; %f %f (%f %f) %i\n", theta, phi, laserDist, x, y, dstX, dstY, occupied);
			if(occupied) cellGrid->informHit(x, y);
			else cellGrid->informMiss(x, y);
		}		
		cellGrid->informHit(dstX, dstY);
		idx++;
	}
}


void printFormatAndExit(void){
	printf("Format:\n"); 
	printf("\trosrun tp1 control [origin x] [origin y] [max x] [max y] [Sensitivity] [Drive-P] [RRT-dist]\n");
	exit(-1);
}
int main(int argc, char **argv){
	//srand((int)time(NULL));
	
	// Sanitize inputs
	//      Label inputs
	const char * argOriginX = argv[1];
	const char * argOriginY = argv[2];
	const char * argMaxX = argv[3];
	const char * argMaxY = argv[4];
	const char * argSensitivity = argv[5];
	const char * argP = argv[6];
	const char * argD = argv[7];
	//      Verify arg-count
	const size_t expectedParams = 7;	
	if(argc <= 1) printFormatAndExit();
	if(argc != expectedParams + 1) {
		fprintf(stderr, "ERROR: Expected %zu params, got %i.\n", expectedParams, argc - 1);
		printFormatAndExit();
	}
	//      Decode initial position
	originX = atof(argOriginX);
	originY = atof(argOriginY);		
	//      Decode grid size
	float maxX = atof(argMaxX);
	float maxY = atof(argMaxY);		
	//       Sensitivity (how close to the target the robot must get before it is considered there)
	double sensitivity = atof(argSensitivity);
	//       Omni-Movement parameters
	double P = atof(argP);
	
	// Initialize ros
	ros::init(argc, argv, "omniBot");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	// Create stage velocity publisher, odometry listener, command listener and, if diffbot, laser listener
	ros::Publisher cmdVelPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10); 
	ros::Subscriber stageOdoSub = n.subscribe("odom", 10, &odomCallback);
	ros::Subscriber laserSub = n.subscribe("base_scan", 10, &laserCallback);

	// Wait until the robot's position is known before proceeding
	while(ros::ok()){
		boost::shared_ptr<const nav_msgs::Odometry> msg = ros::topic::waitForMessage<nav_msgs::Odometry>("odom", n, ros::Duration(10));
		if(msg != boost::shared_ptr<const nav_msgs::Odometry>()){
			odomCallback(*msg);	
			break;
		} else loop_rate.sleep();		
	}	
	printf("Got initial position (%lf, %lf, %lf)\n", px, py, theta);

	// Create the probabilistic cell grid
	const float scale = 1.0;	
	cellGrid = new probabilisticCellGrid_t(maxX, maxY, scale);
	
	// Auxiliaries used in the pathing
	float *pathX = NULL, *pathY = NULL;
	size_t pathCnt = 1;
	size_t pathi = 0;
	float goalX = px, goalY = py, goalTheta = theta;

	// Inform the cell grid that the current cell is unnocupied (since the robot is in it)
	cellGrid->informMiss(px, py);


	while(ros::ok()){
	
		// If we've reached the end of our path, compute a new one
		if (pathi + 1 >= pathCnt){

			// Ensure we're pointed the right way
			const float epsilon = 5*M_PI/180;
			while(fabs(goalTheta - theta) > epsilon){
				geometry_msgs::Twist cmdvel;
				cmdvel.angular.z = (goalTheta > theta) - (goalTheta < theta);			
				cmdVelPub.publish(cmdvel);
				ros::spinOnce();
				loop_rate.sleep();
			}

			// Accept lasers output
			isStopped = true;
			for(size_t repetitions = 0; repetitions < 10; repetitions++){
				ros::spinOnce();
				loop_rate.sleep();
			}
			isStopped = false;
			
			// Load new path
			if (pathX != NULL) free(pathX);
			if (pathY != NULL) free(pathY);
			cellGrid->getPathToClosestUnknownCell(px, py, pathX, pathY, pathCnt);
			pathi = 0;

			// If the path has 0 elements, then we are done exploring
			if (pathCnt == 0){
				printf("Done exploring!\n");
				break;
			}
			assert(pathCnt > 1 && "The current position of the robot is assumed to be known!");

			// Remove the last element so the robot pauses before the last element, in case it's a trap
			pathCnt = std::max<size_t>(2, pathCnt - 1);


			// Compute theta so the robot is looking towards the desired cell at the end
			if (pathX[pathCnt - 1] > pathX[pathCnt - 2]) goalTheta = 0;
			else if (pathX[pathCnt - 1] < pathX[pathCnt - 2]) goalTheta = -M_PI;
			else if (pathY[pathCnt - 1] > pathY[pathCnt - 2]) goalTheta = M_PI / 2;
			else if (pathY[pathCnt - 1] < pathY[pathCnt - 2]) goalTheta = -M_PI / 2;
		}

		float ex = pathX[pathi] - px;
		float ey = pathY[pathi] - py;
		float etheta = goalTheta - theta;

		// If we've reached the goal of the current segment, then go to the next segment
		if (pow(ex, 2) + pow(ey, 2) < pow(sensitivity, 2)){
			printf("Reached goal (%f, %f)\n", pathX[pathi], pathY[pathi]);
			pathi++;
		// Otherwise, just apply velocity
		} else {

			geometry_msgs::Twist cmdvel;
			float velx = pathX[pathi] - px;
			float vely = pathY[pathi] - py;
			cmdvel.linear.x =  velx*cos(theta) + vely*sin(theta);
			cmdvel.linear.y = -velx*sin(theta) + vely*cos(theta);
			cmdvel.angular.z = goalTheta - theta;			
			cmdVelPub.publish(cmdvel);

			cellGrid->printStates();
			printf("x = %f (%f), y = %f (%f), theta = %f (%f)\n", px, goalX, py, goalY, theta, goalTheta);
		}

		// Sleep a bit
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	// Free resources and exit
	delete cellGrid;
	if (pathX != NULL) free(pathX);
	if (pathY != NULL) free(pathY);			
	printf("Exitting...\n");
	return 0;
}


