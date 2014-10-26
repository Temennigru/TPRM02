#ifndef _OCCUPANCYGRID_H
#define _OCCUPANCYGRID_H

#include "PGM.h"
#include <cstdlib>
#include <assert.h>
#include <map>
#include <list>

typedef enum{CLEAR, SOLID, UNKNOWN} cellState_t;

struct cell_t{
	size_t x, y;
	cellState_t state;
};

class cellGrid_t{
	cell_t ** cell;
	size_t w, h;

public:
	size_t getWidth(void){
		return w;
	}
	size_t getHeight(void){
		return h;
	}

	cellGrid_t(size_t w, size_t h) : w(w), h(h){
		cell = (cell_t**)malloc(h*sizeof(cell_t*));
		for (size_t i = 0; i < h; i++) cell[i] = (cell_t*)malloc(w*sizeof(cell_t));
		for (size_t i = 0; i < h; i++){
			for (size_t j = 0; j < w; j++){
				cell[i][j].y = i;
				cell[i][j].x = j;
				cell[i][j].state = UNKNOWN;
			}
		}
	}
	~cellGrid_t(void){
		for (size_t i = 0; i < h; i++) free(cell[i]);
		free(cell);
	}
	void setCellState(size_t x, size_t y, cellState_t state){
		assert(x < w && y < h && "Access must be within bounds!");
		cell[y][x].state = state;
	}
	void getPathToClosestUnknownCell(size_t srcX, size_t srcY, size_t * &x, size_t * &y, size_t &cnt){
		
		// Do a BFS for a cell that has its state set to UNKNOWN
		cell_t * srcCell = &cell[srcY][srcX];
		cell_t * dstCell = NULL;
		std::map<cell_t*, cell_t*> parent;
		std::list<cell_t*> frontier;
		frontier.push_back(srcCell);
		parent[srcCell] = NULL;
		while (!frontier.empty()){

			// Retreive the top of the frontier
			cell_t * elem = frontier.front();
			frontier.pop_front();

			// Check if it is unknown (meaning the destination was reached)
			if (elem->state == UNKNOWN){
				dstCell = elem;
				break;
			}

			// Otherwise, add its children to the search
			for (size_t i = 0; i < 4; i++){
				
				// Compute the child for this instance
				cell_t * child;
				switch (i){
					case 0:  
						if (elem->x == 0) continue; 
						child = &cell[elem->y][elem->x - 1]; break;
					case 1:  
						if (elem->x == w - 1) continue; 
						child = &cell[elem->y][elem->x + 1]; break;
					case 2:  
						if (elem->y == 0) continue; 
						child = &cell[elem->y - 1][elem->x]; break;
					default: 
						if (elem->y == h - 1) continue; 
						child = &cell[elem->y + 1][elem->x]; break;
				}

				// Ignore children that have already been analyzed or that are solid
				if (child->state == SOLID || parent.count(child) != 0) continue;

				// Register this node as the parent of the child and add it to the frontier
				parent[child] = elem;
				frontier.push_back(child);
			}
		}

		// Check if no solution was found
		if (dstCell == NULL){
			x = y = NULL;
			cnt = 0;
			return;
		}

		// Store the solution found
		//       Compute the depth of the solution
		cnt = 0;
		for (cell_t * auxCell = dstCell; auxCell != NULL; auxCell = parent[auxCell]) cnt++;
		//       Store the solution
		x = (size_t*)malloc(cnt*sizeof(size_t));
		y = (size_t*)malloc(cnt*sizeof(size_t));
		size_t auxCnt = cnt - 1;
		for (cell_t * auxCell = dstCell; auxCell != NULL; auxCell = parent[auxCell]){
			x[auxCnt] = auxCell->x;
			y[auxCnt] = auxCell->y;
			auxCnt--;
		}
	}
	void print(void){
		for (size_t y = 0; y < h; y++){
			for (size_t x = 0; x < w; x++){
				switch (cell[y][x].state){
				case CLEAR: putchar('.'); break;
				case SOLID: putchar('#'); break;
				case UNKNOWN: putchar('?'); break;
				}
			}
			putchar('\n');
		}
	}
};

class probabilisticCellGrid_t{
	size_t ** hits;
	size_t ** misses;
	cellGrid_t cellGrid;
	size_t scale;

	// Convert between floating point inputs to grid indices
	void floatPoint2IntegerPoint(float x, float y, size_t &x_st, size_t &y_st){
		x_st = (size_t)(x*scale);
		y_st = (size_t)(y*scale);
	}
	void checkedFloatPoint2IntegerPoint(float x, float y, size_t &x_st, size_t &y_st){
		floatPoint2IntegerPoint(x, y, x_st, y_st);
		assert(x_st < cellGrid.getWidth() && y_st < cellGrid.getHeight() && "Specified index is out of bounds!");
	}

	// Processes a scan event, updating the state of the cells in the non-probabilistic grid
	#define OCCUPANCY_THRESHOLD 0.00
	void informEvent(float x, float y, bool hit){
		size_t x_st, y_st;
		floatPoint2IntegerPoint(x, y, x_st, y_st);
		if (x_st < cellGrid.getWidth() && y_st < cellGrid.getHeight()){
			if (hit) hits[y_st][x_st]++;
			else misses[y_st][x_st]++;
			float occupancyProb = getOccupancyProbability(x, y);
			if (occupancyProb == -1) cellGrid.setCellState(x_st, y_st, UNKNOWN);
			else if (occupancyProb <= OCCUPANCY_THRESHOLD) cellGrid.setCellState(x_st, y_st, CLEAR);
			else cellGrid.setCellState(x_st, y_st, SOLID);
		}
	}


public:

	// Get functions
	size_t getWidth(void){
		return cellGrid.getWidth();
	}
	size_t getHeight(void){
		return cellGrid.getHeight();
	}
	float getOccupancyProbability(float x, float y){
		size_t x_st, y_st;
		floatPoint2IntegerPoint(x, y, x_st, y_st);
		size_t hitCnt = hits[y_st][x_st], missCnt = misses[y_st][x_st];
		size_t Cnt = hitCnt + missCnt;
		if (Cnt == 0) return -1;
		else return (float)hitCnt / (hitCnt + missCnt);
	}

	// Constructor/Destructor (starts with all cells at 0/0 counts)
	probabilisticCellGrid_t(float w, float h, size_t scale) : 
		cellGrid((size_t)(w*scale), (size_t)(h*scale)), scale(scale)
	{
		hits = (size_t**)malloc(cellGrid.getHeight()*sizeof(size_t*));
		misses = (size_t**)malloc(cellGrid.getHeight()*sizeof(size_t*));
		for (size_t i = 0; i < cellGrid.getHeight(); i++){
			hits[i] = (size_t*)calloc(cellGrid.getWidth(), sizeof(size_t));
			misses[i] = (size_t*)calloc(cellGrid.getWidth(), sizeof(size_t));
		}
	}
	~probabilisticCellGrid_t(void){
		for (size_t i = 0; i < cellGrid.getHeight(); i++){
			free(hits[i]);
			free(misses[i]);
		}
		free(hits);   
		free(misses); 
	}

	// Interface for scan event reporting
	void informHit(float x, float y){
		informEvent(x, y, true);
	}
	void informMiss(float x, float y){
		informEvent(x, y, false);
	}

	// Computes the path, passing through the center of the cells, to the clossest cell labeled as unknown
	void getPathToClosestUnknownCell(float srcX, float srcY, float * &x, float * &y, size_t &cnt){

		// Convert floating point source to integer source
		size_t srcX_st, srcY_st;
		checkedFloatPoint2IntegerPoint(srcX, srcY, srcX_st, srcY_st);
		
		// Run the search on the cell grid and copy the results to the floating point result
		size_t * x_st, * y_st;
		cellGrid.getPathToClosestUnknownCell(srcX_st, srcY_st, x_st, y_st, cnt);
		x = (float*)malloc(cnt*sizeof(float));
		y = (float*)malloc(cnt*sizeof(float));
		for (size_t i = 0; i < cnt; i++){
			x[i] = (float)x_st[i] + 1.0 / (2 * scale);
			y[i] = (float)y_st[i] + 1.0 / (2 * scale);
		}

		// Free the integer path produced by cellGrid
		free(x_st);
		free(y_st);
	}

	void printStates(void){
		cellGrid.print();
	}
	void printPGM(const char * FName){
		uint16_t ** pixels = (uint16_t**)malloc(cellGrid.getHeight()*sizeof(uint16_t*));		
		const uint16_t pixelMax = 1024;		
		for(size_t y = 0; y < cellGrid.getHeight(); y++){
			pixels[y] = (uint16_t*)malloc(cellGrid.getWidth()*sizeof(uint16_t));
			for(size_t x = 0; x < cellGrid.getWidth(); x++){
				float occupancyProb = getOccupancyProbability(x, y);;
				if(occupancyProb == -1) pixels[y][x] = 0.5*pixelMax;
				else pixels[y][x] = (1 - occupancyProb)*pixelMax; 		
			}		
		}

		makePGM(FName, pixels, pixelMax, cellGrid.getWidth(), cellGrid.getHeight());

		for(size_t y = 0; y < cellGrid.getHeight(); y++) free(pixels[y]);
		free(pixels);
		
	}
};

#endif

