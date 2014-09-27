#include <cstdlib>
#include <algorithm>
#include <cassert>
#include <cmath>
#include "MovementRestrictions.h"
#include "SegmentIntersect.h"
#include "PGM.h"

MovementRestrictions_t::MovementRestrictions_t(const char * PGMFileName, double paddingRadius){

	// Loads a PGM image from disk
	uint16_t ** pixels, maxValue;
	loadPGM(PGMFileName, pixels, maxValue, width, height);

	// Padds the borders on the image file; bool values represent passable or non-passable
	opaqueCells = (bool**)malloc(height*sizeof(bool*));
	for (size_t i = 0; i < height; i++) opaqueCells[i] = (bool*)malloc(width*sizeof(bool));
	for (size_t i = 0; i < height; i++){
		for (size_t j = 0; j < width; j++){
			opaqueCells[i][j] = false;
			size_t xmin = (size_t)std::max<int>(((int)j - (int)ceil(paddingRadius)), 0);
			size_t xmax = std::min<size_t>(j + (size_t)ceil(paddingRadius), width - 1);
			size_t ymin = (size_t)std::max<int>(((int)i - (int)ceil(paddingRadius)), 0);
			size_t ymax = std::min<size_t>(i + (size_t)ceil(paddingRadius), width - 1);
			for (size_t y = ymin; y <= ymax && !opaqueCells[i][j]; y++){
				for (size_t x = xmin; x <= xmax && !opaqueCells[i][j]; x++){
					if ((y - i)*(y - i) + (x - j)*(x - j) < paddingRadius*paddingRadius && pixels[y][x] != maxValue) opaqueCells[i][j] = true;
				}
			}
		}
	}

	// Free the original image
	for (size_t i = 0; i < height; i++) free(pixels[i]);
	free(pixels);

	// Find the borders of the obstacles in the image; A border is an ocupied nodes with at least one unoccupied 8-neighbor
	bool ** borderCells = (bool**)malloc(height*sizeof(bool*));
	for (size_t i = 0; i < height; i++) borderCells[i] = (bool*)malloc(width*sizeof(bool));
	for (size_t i = 0; i < height; i++){
		for (size_t j = 0; j < width; j++){
			borderCells[i][j] = opaqueCells[i][j] && (
				(j == 0 || !opaqueCells[i][j - 1]) ||
				(j == width - 1 || !opaqueCells[i][j + 1]) ||
				(i == 0 || !opaqueCells[i - 1][j]) ||
				(i == height - 1 || !opaqueCells[i + 1][j]) ||
				!opaqueCells[i - 1][j - 1] ||
				!opaqueCells[i - 1][j + 1] ||
				!opaqueCells[i + 1][j - 1] ||
				!opaqueCells[i + 1][j + 1]
				);
		}
	}

	// Generate a segment list
	//      Count the total number of segments
	segmentCnt = 0;
	for (size_t i = 0; i < height; i++){
		for (size_t j = 0; j < width; j++){
			if (borderCells[i][j]){
				bool rEdge = (j == 0 || !borderCells[i][j - 1]) && ((j <  width - 1) && borderCells[i][j + 1]);
				bool dEdge = (i == 0 || !borderCells[i - 1][j]) && ((i < height - 1) && borderCells[i + 1][j]);
				segmentCnt += (rEdge ? 1 : 0) + (dEdge ? 1 : 0);
			}
		}
	}
	//      Allocate space for the segments
	segmentPx = (size_t*)malloc(segmentCnt*sizeof(size_t*));
	segmentPy = (size_t*)malloc(segmentCnt*sizeof(size_t*));
	segmentQx = (size_t*)malloc(segmentCnt*sizeof(size_t*));
	segmentQy = (size_t*)malloc(segmentCnt*sizeof(size_t*));
	//      Compute the segments, first vertical ones, then horizontal ones
	size_t segmentIdx = 0;
	for (size_t j = 0; j < width; j++){
		for (size_t i = 0; i < height; i++){
			if (!borderCells[i][j]) continue;
			if ((i == 0 || !borderCells[i - 1][j]) && ((i < height - 1) && borderCells[i + 1][j])){
				segmentPx[segmentIdx] = j;
				segmentPy[segmentIdx] = i;
			}
			if ((i > 0 && borderCells[i - 1][j]) && (i == height - 1 || !borderCells[i + 1][j])){
				segmentQx[segmentIdx] = j;
				segmentQy[segmentIdx] = i;
				segmentIdx++;
			}
		}
	}
	for (size_t i = 0; i < height; i++){
		for (size_t j = 0; j < width; j++){
			if (!borderCells[i][j]) continue;
			if ((j == 0 || !borderCells[i][j - 1]) && ((j <  width - 1) && borderCells[i][j + 1])){
				segmentPx[segmentIdx] = j;
				segmentPy[segmentIdx] = i;
			}
			if ((j > 0 && borderCells[i][j - 1]) && (j == width - 1 || !borderCells[i][j + 1])){
				segmentQx[segmentIdx] = j;
				segmentQy[segmentIdx] = i;
				segmentIdx++;
			}
		}
	}

	// Free the border cell array and exit
	for (size_t i = 0; i < height; i++) borderCells[i];
	free(borderCells);
}

MovementRestrictions_t::~MovementRestrictions_t(void){
	for (size_t i = 0; i < height; i++) free(opaqueCells[i]);
	free(opaqueCells);
	free(segmentPx);
	free(segmentPy);
	free(segmentQx);
	free(segmentQy);
}

// Returns true if there are no obstacles between the source and the destination
bool MovementRestrictions_t::isUnobstructed(double srcX, double srcY, double dstX, double dstY){
	if (!isUnocupied((srcX + dstX) / 2, (srcY + dstY) / 2)) return false;
	for (size_t i = 0; i < segmentCnt; i++){
		
		// Skip this edge if any of the current nodes are involved
		if ((segmentPx[i] == srcX && segmentPy[i] == srcY) || (segmentPx[i] == dstX && segmentPy[i] == dstY) ||
			(segmentQx[i] == srcX && segmentQy[i] == srcY) || (segmentQx[i] == dstX && segmentQy[i] == dstY)) continue;

		if (doIntersect<double>(
			srcX, srcY, dstX, dstY,
			segmentPx[i], segmentPy[i], segmentQx[i], segmentQy[i],
			false
			)) return false;
	}
	return true;
}

// Returns true if the referenced position is unocupied in the opacity graph
bool MovementRestrictions_t::isUnocupied(double x, double y){
	size_t x_s = (size_t)x, y_s = (size_t)y;
	assert(x_s < width && y_s < height && "Specified cell must be within the bounds of the image!");
	return !opaqueCells[y_s][x_s];
}


// Returns an array containing the coordinates of each distinct node in the graph
void MovementRestrictions_t::getNodes(uint16_t * &x, uint16_t * &y, size_t &cnt){
	bool ** incidence = (bool**)malloc(height*sizeof(bool*));
	for (size_t i = 0; i < height; i++) incidence[i] = (bool*)calloc(width, sizeof(bool));
	for (size_t i = 0; i < segmentCnt; i++) 
		incidence[segmentPy[i]][segmentPx[i]] = incidence[segmentQy[i]][segmentQx[i]] = true;
	cnt = 0;
	for (size_t i = 0; i < height; i++) for (size_t j = 0; j < width; j++) cnt += incidence[i][j] ? 1 : 0;	
	x = (uint16_t*)malloc(cnt*sizeof(uint16_t));
	y = (uint16_t*)malloc(cnt*sizeof(uint16_t));
	size_t idx = 0;
	for (size_t i = 0; i < height; i++){
		for (size_t j = 0; j < width; j++){
			if (incidence[i][j]){
				x[idx] = j;
				y[idx] = i;
				idx++;
			}
		}
	}
}