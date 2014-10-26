#ifndef _OCCUPANCYGRID_H
#define _OCCUPANCYGRID_H

#include <stdint.h>
#include <cassert>
#include <cfloat>
#include <algorithm>
#include "PGM.h"

class OccupancyGrid_t{

	float occupancyThreshold;
	size_t ** hits;
	size_t ** misses;
	size_t w;
	size_t h;
	size_t scale;

	#define PADDING 0.8
	float padding;
	bool isOpaque(float x, float y){
		float occupancyProb = getOccupancyProbability(x, y);
		if(occupancyProb >= occupancyThreshold) return true;
		else if(occupancyProb < 0) return true;
		else return false;
	}

public:

	// Constructor/Destructor
	OccupancyGrid_t(const size_t w, const size_t h, const size_t scale = 2, const float occupancyThreshold = 0.05) : w(w), h(h), scale(scale), padding(PADDING) {
		
		// Ensure the occupancy threshold is within acceptable values
		assert(occupancyThreshold > 0 && occupancyThreshold <= 1 && "Occupancy must be greater than 0, and less or equal to 1!");
		this->occupancyThreshold = occupancyThreshold;
		
		// Allocate hit/miss array
		hits = (size_t**)malloc(h*scale*sizeof(size_t*));
		misses = (size_t**)malloc(h*scale*sizeof(size_t*));
		for(size_t i = 0; i < h*scale; i++){
			hits[i] = (size_t*)calloc(w*scale, sizeof(size_t));
			misses[i] = (size_t*)calloc(w*scale, sizeof(size_t));
			for (size_t j = 0; j < w*scale; j++) {
				hits[i][j] = 0;
				misses[i][j] = 0;
			}
		}		
	}
	~OccupancyGrid_t(void){
		for(size_t i = 0; i < h*scale; i++){
			free(hits[i]);
			free(misses[i]);
		}
		free(hits);
		free(misses);
	}

	// Get the probability that a cell is unocupied (returns -1.0 if it is unknown); 1 is totally occupied, 0 is unocupied
	float getOccupancyProbability(float col, float row){
		assert((size_t) (row*scale) < h*scale && (size_t) (col*scale) < w*scale && "Referenced position must be within bounds!");
		size_t hcnt = hits[(size_t) (row*scale)][(size_t) (col*scale)];
		size_t mcnt = misses[(size_t) (row*scale)][(size_t) (col*scale)];
		size_t count = hcnt + mcnt;
		if(count == 0) return -1.0;
		else return ((float)hcnt)/count;
	}

	// Returns true if there are no obstacles between the source and the destination
	bool isUnobstructed(float srcX, float srcY, float dstX, float dstY){

		if (srcX < 0.0 && srcY < 0.0 && dstX < 0.0 && dstY < 0.0) { return false; }

		const float dist = sqrt((dstX - srcX)*(dstX - srcX) + (dstY - srcY)*(dstY - srcY));
		const float dx = 2*padding*(dstX - srcX)/dist;
		const float dy = 2*padding*(dstY - srcY)/dist;
	
		float x = srcX, y = srcY;
		const float epsilon = std::min(dx, dy)/2.0;
		while((dx < 0 && x > dstX) || (dx >= 0 && x < dstX) || (dy < 0 && y > dstY) || (dy >= 0 && y < dstY)){
			if(!isUnocupied(x, y)) return false;
			x += dx;
			y += dy;
			//printf("asdf\n");
		}
		return true;

		/*float tan = (float)(dstY - srcY)/(dstX - srcX);
		float x = srcX, y = srcY;
		const float epsilon = 0.1;
		while(fabs(x - dstX) < epsilon && fabs(y - dstY) < epsilon){
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
		return true;*/
	}

	// Returns true if the first obstacle between the source and the destination is an unknown cell
	bool intersectsUnknown(float srcX, float srcY, float dstX, float dstY){
		if (srcX < 0.0 && srcY < 0.0 && dstX < 0.0 && dstY < 0.0) { return false; }
		const float dist = sqrt((dstX - srcX)*(dstX - srcX) + (dstY - srcY)*(dstY - srcY));
		const float dx = /*2*padding*/0.3*(dstX - srcX)/dist;
		const float dy = /*2*padding*/0.3*(dstY - srcY)/dist;
	
		float x = srcX, y = srcY;
		const float epsilon = std::min(dx, dy)/2.0;
		while((dx < 0 && x > dstX) || (dx >= 0 && x <= dstX) || (dy < 0 && y > dstY) || (dy >= 0 && y <= dstY)){
			if(getOccupancyProbability(x,y) < 0) return true;
			else if(isOpaque(x, y)) return false;
			x += dx;
			y += dy;
		}
		return false;
	}

	// Use square spiral to search for unknown visible cells close to a given point
	bool closeToUnknown(float srcX, float srcY, float radius) {
		int nmov1 = 0, nmov2 = 1, nmov3 = 0;
		size_t x = srcX*scale;
		size_t y = srcY*scale;
		int mov = 0;
		bool outOfBounds = false;
		while (true) {

			if ((srcX - ((float)x)/scale)*(srcX - ((float)x)/scale) + (srcY - ((float)y)/scale)*(srcY - ((float)y)/scale) <= radius) { // If not out of radius
				if (hits[(size_t) y][(size_t) x] == 0 && misses[(size_t) y][(size_t) x] == 0) { // Unknown
					if (isUnobstructed(srcX, srcY, ((float)x)/scale, ((float)y)/scale)) { return true; }
				}
				outOfBounds = false;
			}

			if (mov == 0) y ++; //definicao de cada movimento
			if (mov == 1) x --;
			if (mov == 2) y --;
			if (mov == 3) x++;
			nmov1 ++;
			if (nmov3 == 2) { //se 2 se a direcao mudou 2 vezes nmov2 ++ e nmov3 reseta
				nmov2 ++;
				nmov3 = 0;
			}
			if (nmov1 == nmov2) { //se o numero de movimentos necessarios for feito nmov1 reseta, a direcao muda e nmov3 ++
				if (mov != 3) mov++;
				else {
					mov = 0; //se mov = 3 mov reseta
					if (outOfBounds) { return false; }
					outOfBounds = true;
				}
				nmov1 = 0;
				nmov3 ++;
			}
		}
	}


	// Returns true if the referenced position is known to be unocupied within a certain padding
	bool isUnocupied(float x, float y){
		for(float px = std::max<float>(x - padding, 0.0); px < std::min<float>(x + padding, w); px += 1.0/scale){
			for(float py = std::max<float>(y - padding, 0.0); py < std::min<float>(y + padding, h); py += 1.0/scale){
				if(sqrt((px - x)*(px - x) + (py - y)*(py - y)) > padding) continue;
				assert(px >= 0 && py >= 0 && "Generated values must be positive!");
				if(isOpaque(px, py)) return false;
			}
		}
		return true;
	}

	// Records a cell as occupied or unoccupied
	void informOccupancy(float x, float y, bool occupied){
		if((size_t)(x*scale) < w*scale && (size_t)(y*scale) < h*scale){
			if(occupied) hits[(size_t) (y*scale)][(size_t) (x*scale)]++;
			else misses[(size_t) (y*scale)][(size_t) (x*scale)]++;
		}
	}

	// Returns an array containing the coordinates of each distinct node in the graph
	void getNodes(size_t * &x, size_t * &y, size_t &cnt);

    size_t getWidth(void) { return w; }
    size_t getHeight(void) { return h; }

	// Exports the visibility to a PGM
	void exportPGM(const char * FName, bool includePadding = false){
		uint16_t ** pixels = (uint16_t**) malloc(h*scale*sizeof(uint16_t*));
		for(size_t i = 0; i < h*scale; i++) pixels[i] = (uint16_t*)malloc(w*scale*sizeof(uint16_t));
		for(size_t y = 0; y < h*scale; y++){
			for(size_t x = 0; x < w*scale; x++){
				if(includePadding) pixels[y][x] = 2 * isUnocupied((float)x/scale, (float)y/scale);
				else pixels[y][x] = 2 * isOpaque((float)x/scale, (float)y/scale);
				if (getOccupancyProbability((float)x/scale, (float)y/scale) == -1.0) pixels[y][x] = 1;
				//printf("%i", pixels[y][x]);
			}
			//printf("\n");
		}
		
		FILE * test = fopen("/home/viki/catkin_ws/src/tp2/test.txt", "wt");
		fprintf(test, "hello!");
		fclose(test);

		/*printf("hits:\n");
		for(size_t y = 0; y < h*scale; y++){
			for(size_t x = 0; x < w*scale; x++){
				printf("%i", hits[y][x] > 0);
			}
			printf("\n");		
		}
		printf("misses:\n");
		for(size_t y = 0; y < h*scale; y++){
			for(size_t x = 0; x < w*scale; x++){
				printf("%i", misses[y][x] > 0);
			}
			printf("\n");		
		}*/
		makePGM(FName, pixels, 2, w*scale, h*scale);
		for(size_t i = 0; i < h*scale; i++) free(pixels[i]);
		free(pixels);
	}
};

#endif

