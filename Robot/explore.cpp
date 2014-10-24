#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include "../Graph/OccupancyGrid.h"
#include "../Graph/RRT.h"
#include <list>

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

// Occupancy Grid Update
OccupancyGrid_t * grid;
const float rayInc = 0.2;
void laserCallback(const sensor_msgs::LaserScan &msg){	

	// Iterate over each ray, and increment hit/miss grid allong its path 	
	size_t idx = 0;
	for(float phi = msg.angle_min; phi <= msg.angle_max; phi += msg.angle_increment){ 

		float laserDist = msg.ranges[idx];
		if(laserDist <= msg.range_min){
			idx++;
			continue;		
		}	

		// Compute the target x and y
		size_t dstX, dstY;
		if(laserDist >= msg.range_max) dstX = dstY = (size_t)-1;
		else{
			dstX = (size_t) (px + msg.ranges[idx]*cos(phi));
			dstY = (size_t) (py + msg.ranges[idx]*sin(phi));
		}

		/* Runs through all points allong the ray; 
		   Points are incremented roughly proportionally to the size of the intersect between the ray and the cell */
		for(float r = 0; r < laserDist; r += rayInc){
			size_t x = px + r*cos(phi);
			size_t y = py + r*sin(phi);
			grid->informOccupancy(x, y, x == dstX && y == dstY); 		
		}		
		idx++;
	}
}

void printFormatAndExit(void){
	printf("Format:\n"); 
	printf("\trosrun tp1 control [origin x] [origin y] [max x] [max y] [Sensitivity] [Drive-P] [RRT-dist]\n");
	exit(-1);
}
int main(int argc, char **argv){

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

	// Create OccupancyGrid
	grid = new OccupancyGrid_t(maxX, maxY);

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

	// Do full circle in the vicinity of the robot in order to establish sorroundings
	for(int repeat = 0; repeat < 2; repeat++){
		float lastTheta = theta;
		while(ros::ok()){
			
			if(lastTheta > theta) break;		

			geometry_msgs::Twist cmdvel;
			cmdvel.angular.z = 1;
			cmdVelPub.publish(cmdvel);

			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	// Main scan-path-repeat loop		
	//       Error computation	
	uint16_t * pathx, * pathy;
	uint16_t pathi = 0, pathSize = 0;
	float goalTheta;	
	while (ros::ok()){	

		printf("path: %i(%i)\n", pathi, pathSize);

		// Check if the target of the current path has been reached, if so, make a new one
		if(pathi == pathSize){
	
			putchar('.');			

			// Status message informing the end of the current exploration
			if(pathSize != 0){
				printf("Finished exploring (%i, %i)\n", pathx[pathSize - 1], pathy[pathSize - 1]);
				free(pathx);
				free(pathy);
			}			

			// Compute a new path to a cell that is reachable and unknown
			RRT_t RRT(grid);
			RRT.findPath((uint16_t)px, (uint16_t)py, pathx, pathy, pathSize);
			for(size_t i = 0; i < pathSize; i++){
				printf("%zu\t%i %i\n", i, (int)pathx[i], (int)pathy[i]);		
			}			
			assert(pathSize > 0 && "Expected RRT path with at least a destination!");			
			pathi = 0;
			pathSize--;			
			if(pathSize > 0) goalTheta = atan(((float)pathy[pathSize] - (float)pathy[pathSize - 1])/((float)pathx[pathSize] - (float)pathx[pathSize - 1]));
			else goalTheta = atan(((float)pathy[pathSize] - (float)py)/((float)pathx[pathSize - 1] - (float)px));
			
			// Status message informing the start of a new exploration
			printf("Started exploring (%i, %i)\n", pathx[pathSize], pathy[pathSize]);
			
		} else{		

			// Compute proportional error
			float ex = pathx[pathi] - px;
			float ey = pathy[pathi] - py;
			float etheta = goalTheta - theta;
			
			// Check if the new error is within acceptible bounds, if so, load the next goal
			if(ex*ex + ey*ey < sensitivity*sensitivity && (pathi != pathSize - 1 || fabs(etheta) < 45*M_PI/180)){
				pathi++;
				
			// Otherwise, update velocities		
			} else {

				// Computes the new (x,y) velocities in universal coordinates
				double universal_velX = P*ex;
				double universal_velY = P*ey;

				// Computes the velocity adjusted for theta
				double local_velX = universal_velX*cos(theta) + universal_velY*sin(theta);
				double local_velY = -universal_velX*sin(theta) + universal_velY*cos(theta);			

				// Generate message for stage and publish it
				geometry_msgs::Twist cmdvel;
				cmdvel.linear.x = local_velX;
				cmdvel.linear.y = local_velY;
				cmdvel.angular.z = P*etheta;
				
				//printf("theta: %lf; goalTheta: %lf; etheta: %lf\n", theta, goalTheta, etheta);
				printf("x: %lf(%lf), y: %lf(%lf), theta: %lf(%f)\n", px, (double)pathx[pathi], py, (double)pathy[pathi], theta, goalTheta);

				// Publish new velocity to stage control
				cmdVelPub.publish(cmdvel);
			}
		}

		// Sleep a bit
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	// Free resources and exit
	delete grid;
	printf("Exitting...\n");
	return 0;
}
