#ifndef _OCCUPANCYGRID_H
#define _OCCUPANCYGRID_H

#include <stdint.h>
#include <cassert>
#include <cfloat>
#include <algorithm>

class OccupancyGrid_t{

	float occupancyThreshold;
	size_t ** hits;
	size_t ** misses;
	size_t w;
	size_t h;
	
	const float padding = 0.5;
	bool isOpaque(double x, double y){
		return getOccupancyProbability(y, x) >= occupancyThreshold;
	}

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
		assert(row < h && col < w && "Referenced position must be within bounds!");
		size_t count = hits[row][col] + misses[row][col];
		if(count == 0) return -1.0;
		else return hits[row][col]/count;
	}

	// Returns true if there are no obstacles between the source and the destination
	bool isUnobstructed(size_t srcX, size_t srcY, size_t dstX, size_t dstY){
		float tan = (float)(dstY - srcY)/(dstX - srcX);
		float x = srcX, y = srcY;
		while((size_t)x != dstX && (size_t)y != dstY){
			if(!isUnocupied(x, y)) return false;
			float x_dx = floor(x + 1) - x, x_dy = x_dx*tan;
			float y_dy = floor(y + 1) - y, y_dx = y_dy/tan;
			if(x_dx == y_dx){
				if(x_dy < y_dy){
					x += x_dx;
					y += x_dy;				
				} else {
					x += y_dx;
					y += y_dy;							
				}			
			} else if(x_dx < y_dx){
				x += x_dx;
				y += x_dy;
			} else{
				x += y_dx;
				y += y_dy;			
			}
		}				
		return true;			
	}

	// Returns true if the referenced position is known to be unocupied 
	bool isUnocupied(double x, double y){
		for(size_t px = (size_t)std::max<double>(x - padding, 0.0); px < std::min<double>(x + padding, w); px++){
			for(size_t py = (size_t)std::max<double>(y - padding, 0.0); py < std::min<double>(y + padding, h); py++){
				if((px - x)*(px - x) + (py - y)*(py - y) < padding) continue;
				if(isOpaque(px, py)) return false;			
			}
		}
		return true;
	}

	// Records a cell as occupied or unoccupied
	void informOccupancy(size_t x, size_t y, bool occupied){
		if(x < w && y < h){
			if(occupied) hits[y][x]++;
			else misses[y][x]++;
		}
	}

	// Returns an array containing the coordinates of each distinct node in the graph
	void getNodes(size_t * &x, size_t * &y, size_t &cnt);

    size_t getWidth(void) { return w; }
    size_t getHeight(void) { return h; }
};

#endif

