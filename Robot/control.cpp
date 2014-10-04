#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <list>
#include <tp1/ccoord.h>

// Coordinate instructions sent to the robot
//       The current objective
volatile double goalx;
volatile double goaly;
//       Future objectives
std::list<tp1::ccoord> goalQueue;
void goalCallback(const tp1::ccoord &msg){
	printf("told to go to (%lf, %lf)\n", msg.x, msg.y);
	goalQueue.push_front(msg);	
}
void loadNextGoal(void){

	// If the queue is empty, don't update the goal
	if(goalQueue.empty()) return;

	// Otherwise, pop use the top element as the goal and pop it
	const tp1::ccoord &msg = goalQueue.front();
	goalx = msg.x;
	goaly = msg.y;
	goalQueue.pop_front();
	printf("going to (%lf, %lf)\n", goalx, goaly);
}

// Position of the robot
volatile double px;
volatile double py;
volatile double theta;
void odomCallback(const nav_msgs::Odometry &msg){
	px = msg.pose.pose.position.x;
	py = msg.pose.pose.position.y;
	theta = asin(msg.pose.pose.orientation.z)*2;
}

// Lazorzzz
double KR, KA, KL, KW;
volatile double v;
volatile double omega;
void laserCallback(const sensor_msgs::LaserScan &msg){

	// Compute the attraction force
	float d = sqrt((px - goalx)*(px - goalx) + (py - goaly)*(py - goaly));
	float csi = atan2(goaly - py, goalx - px) - theta;
	float fxa = KA*d*cos(csi);
	float fya = KA*d*sin(csi);

	// Compute the repulsive force
	float fxr = 0, fyr = 0;
	size_t idx = 0;
	const float rangeIgnore = 5;
	for(float phi = msg.angle_min; phi <= msg.angle_max; phi += msg.angle_increment){ 
		//if(phi < -M_PI*45.0/180 || phi > M_PI*45.0/180) continue;		
		float r = msg.ranges[idx++];
		if(r <= msg.range_min || r >= msg.range_max/* || r >= rangeIgnore || r >= d*/) continue;		
		fxr += -KR*cos(phi)/std::max(r - 1.0, 0.01);		
		fyr += -KR*sin(phi)/std::max(r - 1.0, 0.01);
		//fxr += -KR*sin(phi)/std::max(r - 0.75, 0.01);		
		//fyr += -KR*cos(phi)/std::max(r - 0.75, 0.01);
		//printf("(%i) laser got (%lf,%lf)\n", idx, phi, r);		
	}
	
	
	// Resulting force and robot movement
	//printf("fxr %lf, fyr %lf\n", fxr, fyr);
	float fx = fxa + fxr;
	float fy = fya + fyr;
	v = KL*(fx*cos(theta) + fy*sin(theta));
	omega = KW*(atan2(fy, fx) - theta);	
}

void printFormatAndExit(void){
	printf("Format:\n"); 
	printf("\t(1) rosrun tp1 control [Sensitivity] \"omni\" [KP] [KD] [KI] \n");
	printf("\t(2) rosrun tp1 control [Sensitivity] \"diff\" [KR] [KA] [KL] [KW]\n");
	exit(-1);
}
int main(int argc, char **argv){

	// Sanitize inputs
	//      Label inputs
	const char * argSensitivity = argv[1];
	const char * argDriveType = argv[2];
	const char * argKP = argv[3];
	const char * argKD = argv[4];
	const char * argKI = argv[5];
	const char * argKR = argv[3];
	const char * argKA = argv[4];
	const char * argKL = argv[5];
	const char * argKW = argv[6];
	//      Verify arg-count
	const size_t expectedParamsDiff = 6;	
	const size_t expectedParamsOmni = 5;	
	if(argc <= 1) printFormatAndExit();
	if(argc != expectedParamsDiff + 1 && argc != expectedParamsOmni + 1) {
		fprintf(stderr, "ERROR: Expected %zu or %zu params, got %i.\n", expectedParamsDiff, expectedParamsOmni, argc - 1);
		printFormatAndExit();
	}
	//      Decode drive type
	enum {DIFF_DRIVE, OMNI_DRIVE} driveType;
	if(strcmp(argDriveType, "diff") == 0){
		driveType = DIFF_DRIVE;	
		if(argc != expectedParamsDiff + 1){
			fprintf(stderr, "ERROR: Expected %zu params, got %i.\n", expectedParamsDiff, argc - 1);
			printFormatAndExit();					
		}
	} else if(strcmp(argDriveType, "omni") == 0){
		driveType = OMNI_DRIVE;	
		if(argc != expectedParamsOmni + 1){
			fprintf(stderr, "ERROR: Expected %zu params, got %i.\n", expectedParamsOmni, argc - 1);
			printFormatAndExit();					
		}
	} else{
		fprintf(stderr, "ERROR: Expected \"omni\" or \"diff\" drive types, got \"%s\"\n", argDriveType);
		printFormatAndExit();
	}		
	//       Sensitivity (how close to the target the robot must get before it is considered there)
	double sensitivity = atof(argSensitivity);
	//       Omni-Movement parameters
	double KP, KD, KI, dex, dey, ex, ey, iex, iey;
	if(driveType == OMNI_DRIVE){
		KP = atof(argKP);
		KD = atof(argKD);
		KI = atof(argKI);
	//       Diff-Movement parameters
	} else{
		KR = atof(argKR);
		KA = atof(argKA);
		KL = atof(argKL);
		KW = atof(argKW);
		printf("KR: %lf, KA: %lf, KL: %lf, KW: %lf\n", KR, KA, KL, KW);
	}

	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, (driveType == OMNI_DRIVE) ? "omniBot" : "diffBot");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	// Create stage velocity publisher, odometry listener, command listener and, if diffbot, laser listener
	ros::Publisher cmdVelPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10); 
	ros::Subscriber stageOdoSub = n.subscribe("odom", 10, &odomCallback);
	ros::Subscriber cmdPosSub = n.subscribe("cmd_pos", 10, &goalCallback);
	ros::Subscriber laserSub;
	if(driveType == DIFF_DRIVE) laserSub = n.subscribe("base_scan",10, &laserCallback);

	// Wait until the robot's position is known before proceeding (uses scoping to destroy subscriber once it's ok to go)
	while(ros::ok()){
		boost::shared_ptr<const nav_msgs::Odometry> msg = ros::topic::waitForMessage<nav_msgs::Odometry>("odom", n, ros::Duration(10));
		if(msg != boost::shared_ptr<const nav_msgs::Odometry>()){
			odomCallback(*msg);	
			break;
		} else loop_rate.sleep();		
	}	
	goalx = px;
	goaly = py;
	printf("Got initial position (%lf, %lf, %lf)\n", px, py, theta);


	// Do path		
	//       Initialize omni-drive PID values	
	if(driveType == OMNI_DRIVE){
		dex = dey = 0;
		ex = ey = 0;
		iex = iey = 0;
	}
	bool moving = true;
	while (ros::ok()){	

		// Omni-drive PID update
		if(driveType == OMNI_DRIVE){	
			dex = (goalx - px) - ex;
			dey = (goaly - py) - ey;
			ex = goalx - px;
			ey = goaly - py;
			iex += ex;
			iey += ey;
			
		// Diff-drive error computation
		} else {
			ex = goalx - px;
			ey = goaly - py;
		}


		// Check if the new error is within acceptible bounds, if so, load the next goal
		if(ex*ex + ey*ey < sensitivity*sensitivity){

			// Inform that the robot has reached it's goals
			if(moving){
				printf("Goal reached within specified bounds.\n");
			}

			// Load the next goal, if one exists
			if(goalQueue.empty()) moving = false;
			else{
				loadNextGoal();
				moving = true;			
			}

			// Reset omni-drive error counter
			if(driveType == OMNI_DRIVE){	
				dex = dey = 0;
				ex = ey = 0;
				iex = iey = 0;
			}

		// Otherwise, update velocities		
		} else {

			// Compute new velocity
			geometry_msgs::Twist cmdvel;
			//       Omni-drive
			if(driveType == OMNI_DRIVE){

				// Computes the new (x,y) velocities in universal coordinates
				double universal_velX = KP*ex + KD*dex + KI*iex;
				double universal_velY = KP*ey + KD*dey + KI*iey;

				// Computes the velocity adjusted for theta
				double local_velX = universal_velX*cos(theta) + universal_velY*sin(theta);
				double local_velY = -universal_velX*sin(theta) + universal_velY*cos(theta);
				
				// Generate message for stage and publish it
				cmdvel.linear.x = local_velX;
				cmdvel.linear.y = local_velY;
			//      Diff-drive
			} else{
				cmdvel.linear.x = v;
				cmdvel.linear.y = 0;		
				cmdvel.angular.z = omega;
			}
		
			// Publish new velocity to stage control
			cmdVelPub.publish(cmdvel);
		}

		// Sleep a bit
		ros::spinOnce();
		loop_rate.sleep();
	}

	printf("Exitting...\n");
	return 0;
}

