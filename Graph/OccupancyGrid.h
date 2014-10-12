#ifndef _OCCUPANCYGRID_H
#define _OCCUPANCYGRID_H

#include <stdint.h>
#include <cassert>

class OccupancyGrid_t{

	size_t ** hits;
	size_t ** misses;
	size_t w;
	size_t h;
	
public:

	// Constructor/Destructor
	OccupancyGrid_t(const size_t w, const size_t h, const float occupancyThreshold = 0.20) : w(w), h(h) {
	
		// Ensure the occupancy threshold is within acceptable values
		assert(occupancyThreshold > 0 && occupancyThreshold <= 1 && "Occupancy must be greater than 0, and less or equal to 1!");
		this->occupancyThreshold = occupancyThreshold;
		
		// Allocate hit/miss array
		hits = (size_t**)malloc(h*sizeof(size_t*));
		misses = (size_t**)malloc(h*sizeof(size_t*));
		for(size_t i = 0; i < h; i++){
			hits[i] = (size_t*)calloc(w, sizeof(size_t));
			misses[i] = (size_t*)calloc(w, sizeof(size_t));
		}		
	}
	~OccupancyGrid_t(void){
		for(size_t i = 0; i < h; i++){
			free(hits[i]);
			free(misses[i]);
		}
		free(hits);
		free(misses);
	}

	// Get the probability that a cell is unocupied (returns -1.0 if it is unknown); 1 is totally occupied, 0 is unocupied
	float getOccupancyProbability(size_t row, size_t col){
		size_t count = hits[row][col] + misses[row][col];
		if(count == 0) return -1.0;
		else return hits[row][col]/count;
	}

	// Returns true if there are no obstacles between the source and the destination
	bool isUnobstructed(float srcX, float srcY, float dstX, float dstY){
				
			
	}

	// Returns true if the referenced position is known to be unocupied 
	bool isUnocupied(double x, double y){
		return getOccupancyProbability(y, x) >= occupancyThreshold;
	}

	// Returns an array containing the coordinates of each distinct node in the graph
	void getNodes(uint16_t * &x, uint16_t * &y, size_t &cnt);

    uint16_t getWidth () { return this->width; }
    uint16_t getHeight () { return this->height; }
};

#endif
