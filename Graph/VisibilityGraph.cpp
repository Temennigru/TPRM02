#include <cstdlib>
#include "VisibilityGraph.h"

// Adds a node to the graph, computing it's visibility towards all other nodes already in the graph
void VisibilityGraph_t::addNode(graph2D::node_t * node){
	for (auto n = G.begin(); n != G.end(); n++) {

		// Add the edge if there is visibility between the two edges
		if (restrictions->isUnobstructed((*n)->x, (*n)->y, node->x, node->y)) 
			graph2D::addEdge((*n), node);
	}

	// Insert the node into the graph-vector
	G.push_back(node);
}

// Constructor/Destructor
VisibilityGraph_t::VisibilityGraph_t(MovementRestrictions_t * restrictions) :
	restrictions(restrictions)
{

	// Retrieve the obstacle node coordinates
	uint16_t * x, *y;
	restrictions->getNodes(x, y, nodeCnt);

	// Allocate and then assign coordinates to graph nodes
	for (size_t i = 0; i < nodeCnt; i++){
		graph2D::node_t * node = new graph2D::node_t();
		node->x = x[i];
		node->y = y[i];
		addNode(node);
	}
	free(x);
	free(y);
}

VisibilityGraph_t::~VisibilityGraph_t(void){
    for (auto node = G.begin(); node != G.end(); node++) delete *node;
}

// Finds a path between two points using the visibility graph
void VisibilityGraph_t::findPath(
	uint16_t srcX, uint16_t srcY, uint16_t dstX, uint16_t dstY,
	uint16_t * &x, uint16_t * &y, uint16_t &cnt
	){
	// Add the source and destination as nodes in the graph
	graph2D::node_t * src = new graph2D::node_t(); src->x = srcX; src->y = srcY;
	graph2D::node_t * dst = new graph2D::node_t(); dst->x = dstX; dst->y = dstY;
	addNode(src);
	addNode(dst);

	// Use A* algorithm to compute the shortest path
	graph2D::solve(src, dst, G);

	// Store the path taken
	//       Count the number of nodes from dst to src
	cnt = 1;
	for (graph2D::node_t * aux = dst; aux->parent != NULL; aux = aux->parent) cnt++;
	//       Allocate memory for steps
	x = (uint16_t*)malloc(cnt*sizeof(uint16_t));
	y = (uint16_t*)malloc(cnt*sizeof(uint16_t));
	//       Store step values
	size_t idx = cnt - 1;
	for (graph2D::node_t * aux = dst; aux != NULL; aux = aux->parent){
		x[idx] = aux->x;
		y[idx] = aux->y;
		idx--;
	}

	// Remove the added edges from the graph
	graph2D::removeNode(src);
	graph2D::removeNode(dst);
	G.pop_back();
	G.pop_back();
	delete src;
	delete dst;
}
