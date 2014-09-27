#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include "VisibilityGraph.h"
#include "MovementRestrictions.h"

int main(int argc, char ** argv){
	const char * PGMFile = "pgm/tp1_floor1.pgm";
	const double safetyRadius = 3.0;

	MovementRestrictions_t restrictions(PGMFile, safetyRadius);
	VisibilityGraph_t V(&restrictions);
	bool asdf = restrictions.isUnobstructed(20, 220, 50, 220);

	uint16_t * pathX, * pathY, pathSize;
	V.findPath(25, 215, 225, 15, pathX, pathY, pathSize);
	//for (size_t i = 0; i < pathSize; i++)
	//	printf("%i, %i\n", pathX[i], pathY[i]);
	//free(pathX);
	//free(pathY);
	return 0;
}