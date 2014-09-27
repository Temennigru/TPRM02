#ifndef _VISIBILITY_GRAPH_H_
#define _VISIBILITY_GRAPH_H_

#include <vector>
#include "MovementRestrictions.h"
#include "Graph2D.h"

class VisibilityGraph_t{

	// The nodes in the graph
	std::vector<graph2D::node_t *> G;
	size_t nodeCnt;

	// The obstacles and restrictions
	MovementRestrictions_t *restrictions;

	// Adds a node to the graph, computing it's visibility towards all other nodes already in the graph
	void addNode(graph2D::node_t * node);

public:

	// Constructor/Destructor
	VisibilityGraph_t(MovementRestrictions_t *restrictions);
	~VisibilityGraph_t(void);

	// Finds a path between two points using the visibility graph
	void findPath(
		uint16_t srcX, uint16_t srcY, uint16_t dstX, uint16_t dstY,
		uint16_t * &x, uint16_t * &y, uint16_t &cnt
	);
};

#endif
