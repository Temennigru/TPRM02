#ifndef _MOVEMENT_RESTRICTIONS_H
#define _MOVEMENT_RESTRICTIONS_H

#include <stdint.h>

class MovementRestrictions_t{

	// The padded passability grid (originally loaded from the pgm) 
	bool ** opaqueCells;
	uint16_t width, height;

	// Segmented representation of the image
	size_t segmentCnt;
	size_t * segmentPx;
	size_t * segmentPy;
	size_t * segmentQx;
	size_t * segmentQy;

public:

	// Constructor/Destructor
	MovementRestrictions_t(const char * PGMFileName = "input.pgm", double paddingRadius = 3.0);
	~MovementRestrictions_t(void);

	// Returns true if there are no obstacles between the source and the destination
	bool isUnobstructed(double srcX, double srcY, double dstX, double dstY);

	// Returns true if the referenced position is unocupied in the opacity graph
	bool isUnocupied(double x, double y);

	// Returns an array containing the coordinates of each distinct node in the graph
	void getNodes(uint16_t * &x, uint16_t * &y, size_t &cnt);

    uint16_t getWidth () { return this->width; }
    uint16_t getHeight () { return this->height; }
};

#endif