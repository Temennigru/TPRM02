#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include "MovementRestrictions.h"
#include "VisibilityGraph.h"
#include "RRT.h"
#include "../Robot/pointPublisher.h"
#include "ros/ros.h"

void printFormatAndExit(void){
	printf("Format:\n");
	printf("	(1) rosrun tp1 path [srcX] [srcY] [dstX] [dstY] [pgm file] [padding] RRT \n");
	printf("	(2) rosrun tp1 path [srcX] [srcY] [dstX] [dstY] [pgm file] [padding] VisibilityGraph\n");
	exit(0);
}

int main(int argc, char ** argv){

	// Initialize ros
	ros::init(argc, argv, "path_designator");		
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	ros::Publisher cmdPosPub = n.advertise<tp1::ccoord>("cmd_pos", 10); 
	sleep(1);	

	// Sanitize inputs
	const char * argSrcX = argv[1];
	const char * argSrcY = argv[2];
	const char * argDstX = argv[3];
	const char * argDstY = argv[4];
	const char * argFName = argv[5];
	const char * argPadding = argv[6];
	const char * argSelAlgo = argv[7];
	const size_t argCnt = 7;
	if(argc == 1) printFormatAndExit();
	else if(argc - 1 != argCnt){
		fprintf(stderr, "ERROR: Expected %zu arguments, got %i.\n", argCnt, argc - 1);
		printFormatAndExit();
	}	
	const double srcX = atof(argSrcX);
	const double srcY = atof(argSrcY);
	const double dstX = atof(argDstX);
	const double dstY = atof(argDstY);
	const char * PGMFile = argFName;
	enum {RRT, VISIBILITY_GRAPH} SELECTED_ALGORITHM;
	if(std::string(argSelAlgo) == std::string("RRT")) SELECTED_ALGORITHM = RRT;
	else if(std::string(argSelAlgo) == std::string("VisibilityGraph")) SELECTED_ALGORITHM = VISIBILITY_GRAPH;
	else{
		fprintf(stderr, "ERROR: Expected algorithm type to be %s or %s, got \"%s\"\n", "RRT", "VisibilityGraph", argSelAlgo);
		printFormatAndExit();
	}
	const double padding = atof(argPadding);

	// Load PGM file, computing restrictions
	MovementRestrictions_t restrictions(PGMFile, padding);

	// Verify if the source and destination nodes are not within solid objects
	if(!restrictions.isUnocupied(srcX, srcY) || !restrictions.isUnocupied(dstX, dstY)){
		fprintf(stderr, "ERROR: Source or destination points are within obstacles or their padding!\n");
		exit(1);
	} else if(srcX < 0 || srcY < 0 || srcX >= restrictions.getWidth() || srcY >= restrictions.getHeight()){
		fprintf(stderr, "ERROR: Source or destination points are out of bounds!\n");
		exit(1);
	}
	

	// Compute path
	uint16_t * pathX, * pathY, pathSize;
	if(SELECTED_ALGORITHM == RRT){
		RRT_t R(&restrictions);
		R.findPath(srcX, srcY, dstX, dstY, pathX, pathY, pathSize);
	} else{
		VisibilityGraph_t V(&restrictions);
		V.findPath(srcX, srcY, dstX, dstY, pathX, pathY, pathSize);
	}
	
	// Add the points in the path to a list, and send it to the controler
	//std::list<tp1::ccoord> path;
	for(size_t i = 0; i < pathSize; i++){
		tp1::ccoord aux; aux.x = pathX[i]/10.0; aux.y = pathY[i]/10.0; aux.theta = 0;
		printf("[%zu] %lf, %lf\n", i, aux.x, aux.y);
		cmdPosPub.publish(aux);
		loop_rate.sleep();		
	}
	//publishPoints(path);

	// Free resources and exit
	free(pathX);
	free(pathY);
	return 0;
}
