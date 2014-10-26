#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include "../Graph/OccupancyGrid.h"
#include "../Graph/RRT.h"
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

// Occupancy Grid Update
bool isStopped = false;
OccupancyGrid_t * grid;
const float rayInc = 0.2;
void laserCallback(const sensor_msgs::LaserScan &msg){	
	
	// If the robot is moving, do not trust its laser
	if(!isStopped) return;

	printf("WHOOP\n");

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
		if(laserDist >= 3.0*msg.range_max/6.0) dstX = dstY = FLT_MAX;
		else{
			dstX = px + msg.ranges[idx]*cos(phi);
			dstY = py + msg.ranges[idx]*sin(phi);
		}

		/* Runs through all points allong the ray; 
		   Points are incremented roughly proportionally to the size of the intersect between the ray and the cell */
		for(float r = 0; r < laserDist; r += rayInc){
			float x = px + r*cos(phi);
			float y = py + r*sin(phi);
			bool occupied = pow(x - dstX, 2) + pow(y - dstY, 2) < 0.05*0.05;
			//sprintf("theta = %f ; phi %f ; r = %f ; %f %f (%f %f) %i\n", theta, phi, laserDist, x, y, dstX, dstY, occupied);
			grid->informOccupancy(x, y, occupied);
		}
		grid->informOccupancy(dstX, dstY, dstX != FLT_MAX);
		idx++;
	}
}


// Rotate in position until all points in a radius are either scanned or unknown
void stopAndScan(OccupancyGrid_t * grid, ros::Publisher * cmdvelPub){
	printf ("Entrei\n");

	ros::Rate loop_rate(10);
	const float rotateAngle = M_PI/5;
	const float scanDist = 2;
	const float scanAngle = M_PI/180;
	float goalTheta = theta + ((theta < 0) ? M_PI : -M_PI);

	while(ros::ok()){

		// Check if we are done scanning
		//printf("%s %i\n", __FILE__, __LINE__);
		bool doneScanning = !grid->closeToUnknown(px, py, scanDist);
		printf("px: %f py: %f scanDist: %f\n", px, py, scanDist);
		//printf("%s %i\n", __FILE__, __LINE__);
		if(doneScanning) {
			//printf("doneScanning\n");
			//getchar();
			break;
		}
		// If not, rotate by rotateAngle
		const float epsilon = 5*M_PI/180;
		float goalTheta = theta + rotateAngle;
		if(goalTheta > M_PI) goalTheta -= 2*M_PI;
		while(fabs(goalTheta - theta) >= epsilon){
			geometry_msgs::Twist cmdvel;
			cmdvel.angular.z = 1;
			cmdvelPub->publish(cmdvel);
			ros::spinOnce();
			loop_rate.sleep();
			printf ("goal theta: %f theta: %f epsilon: %f\n", goalTheta, theta, epsilon);
		}

		// Wait a bit for the laser to scan the environment
		geometry_msgs::Twist cmdvel;
		cmdvelPub->publish(cmdvel);
		loop_rate.sleep();
		isStopped = true;
		ros::spinOnce();
		loop_rate.sleep();
		isStopped = false;
		printf ("dando loop\n");
		grid->exportPGM("/home/viki/catkin_ws/src/tp2/initial.pgm", true);
	}
	printf ("sai\n");
	grid->exportPGM("/home/viki/catkin_ws/src/tp2/initial.pgm", true);
}

void printFormatAndExit(void){
	printf("Format:\n"); 
	printf("\trosrun tp1 control [origin x] [origin y] [max x] [max y] [Sensitivity] [Drive-P] [RRT-dist]\n");
	exit(-1);
}
int main(int argc, char **argv){
	srand((int)time(NULL));
	
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
	//printf("Doing a full circle to get initial bearings...\n");
	
	// Generate a print of the visual field
	grid->exportPGM("/home/viki/catkin_ws/src/tp2/initial.pgm", true);

	// Main scan-path-repeat loop		
	printf("Starting search...\n");
	//       Error computation	
	float * pathx, * pathy, * pathTheta;
	size_t pathi = 0, pathSize = 0;
	float goalTheta;
	while (ros::ok()){

		if (pathi != pathSize && !grid->isUnobstructed(px, py, pathx[pathi], pathy[pathi])){
			pathi = pathSize;
		}	

		//printf("path: %i(%i)\n", pathi, pathSize);

		// Check if the target of the current path has been reached, if so, make a new one
		if(pathi == pathSize){

			grid->exportPGM("/home/viki/catkin_ws/src/tp2/initial.pgm", true);


			// Scan arround the surroundings
			stopAndScan(grid, &cmdVelPub);
	

			// Status message informing the end of the current exploration
			if(pathSize != 0){
				printf("Finished exploring (%f, %f)\n", pathx[pathSize - 1], pathy[pathSize - 1]);
				free(pathx);
				free(pathy);
			}

			// Compute a new path to a cell that is reachable and unknown
			RRT_t RRT(grid);
			RRT.findPath(px, py, pathx, pathy, pathSize);
			printf("Setting new path:\n");
			for(size_t i = 0; i < pathSize; i++){
				printf("%zu\t%f %f\n", i, pathx[i], pathy[i]);
			}
			assert(pathSize > 0 && "Expected RRT path with at least a destination!");
			pathi = 0;

			// Compute theta allong the trajectory
			pathTheta = (float*)malloc(pathSize*sizeof(float));			
			for(size_t i = 0; i < pathSize; i++){
				pathTheta[i] = atan(((float)pathy[i+1] - (float)pathy[i])/((float)pathx[i+1] - (float)pathx[i]));
			}
			
			// Status message informing the start of a new exploration
			printf("Started exploring (%f, %f)\n", pathx[pathSize], pathy[pathSize]);
			
		} else {

			// Compute proportional error
			float ex = pathx[pathi] - px;
			float ey = pathy[pathi] - py;
			float etheta = goalTheta - theta;
			
			// Check if the new error is within acceptible bounds, if so, load the next goal
			if(ex*ex + ey*ey < sensitivity*sensitivity /*&& (pathi != pathSize - 1 || fabs(etheta) < 45*M_PI/180)*/){

				printf("Reached goal (%f, %f)\n", pathx[pathi], pathy[pathi]);

				// Wait for the robot to catch up with where we are going
				/*geometry_msgs::Twist cmdvel;
				cmdvel.linear.x = 0;
				cmdvel.linear.y = 0;
				cmdvel.angular.z = 0.02;//P*etheta;
				while(fabs(theta - pathTheta[pathi]) < 0.01){
					cmdVelPub.publish(cmdvel);
					ros::spinOnce();
					loop_rate.sleep();
				}*/
				stopAndScan(grid, &cmdVelPub);
				if (!grid->isUnocupied(px, py)) { // Is in wall
					printf("Stuck in wall\n");
					pathi--;
				} else {
					// Load next destination, unless we discovered an obstruction
					pathi++;
				}
				grid->exportPGM("/home/viki/catkin_ws/src/tp2/initial.pgm", true);
				
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
				//cmdvel.angular.z = P*etheta;
				
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

