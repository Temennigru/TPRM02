//
//  RRT.cpp
//  TPRM01
//
//  Created by Jean-Luc Nacif Coelho on 9/30/14.
//  Copyright (c) 2014 GrupoRM. All rights reserved.
//

#include "RRT.h"
#include <cstdlib>
#include <cmath>


#define MAX_DIST 4.0f
#define SQ_MAX_DIST 16.0f


// Adds a node to the graph, computing it's visibility towards all other nodes already in the graph
// Return if found dst
bool RRT_t::addNode(graph2D::node_t * &dst){
	
	// Randomply generates a new candidate node
    graph2D::node_t_ptr node = new graph2D::node_t();
    node->x = (graph2D::nodeValue_t)((float)rand()/RAND_MAX)*restrictions->getWidth();
    node->y = (graph2D::nodeValue_t)((float)rand()/RAND_MAX)*restrictions->getHeight();

    double min_dist = 0.0;
    graph2D::node_t_ptr closest_node = NULL;

    // Find closest node
    for (auto n = G.begin(); n != G.end(); n++) {
		double dist = graph2D::sqdistance(*n, node);
        if (closest_node == NULL || dist < min_dist) {
            closest_node = *n;
            min_dist = dist;
        }
    };

    // Fix distance
    if (min_dist > SQ_MAX_DIST) {
        printf("%f %f -> %f %f", node->x, node->y, closest_node->x, closest_node->y);
		node->x = (graph2D::nodeValue_t)( ( (node->x - closest_node->x) / sqrt(graph2D::sqdistance(node, closest_node))) * MAX_DIST) + closest_node->x;
        node->y = (graph2D::nodeValue_t)( ( (node->y - closest_node->y) / sqrt(graph2D::sqdistance(node, closest_node))) * MAX_DIST) + closest_node->y;
        //printf(" (%f %f)\n", node->x, node->y);
		assert(node->x >= 0.0 && node->y >= 0.0 && "Generated point must be within positive bounds!");
    }

    // If the node is visible, add it as a connecting node
    if (restrictions->isUnobstructed(closest_node->x, closest_node->y, node->x, node->y)) {
		graph2D::addEdge(closest_node, node);
        G.push_back(node);
		return false;
	// Otherwise, if the intersecting node is an unknown point, then use it as a destination
    } else if (restrictions->intersectsUnknown(closest_node->x, closest_node->y, node->x, node->y)){
		graph2D::addEdge(closest_node, node);
        G.push_back(node);
		dst = node;
		return true;
	// If neither is the case, the intersect is with a wall, so don't add the edge
	} else{
		delete node;
		return false;
	}
}

// Constructor/Destructor
RRT_t::RRT_t(OccupancyGrid_t * restrictions) : restrictions(restrictions) {}

RRT_t::~RRT_t(void){
    for (auto node = G.begin(); node != G.end(); node++) delete *node;
}

// Finds a path between two points using the visibility graph
void RRT_t::findPath(
     float srcX, float srcY,
     float * &x, float * &y, size_t &cnt
){
    // Add the source and destination as nodes in the graph
    // Dist won't be added as to not mess up the algorithm
    graph2D::node_t * src = new graph2D::node_t(); src->x = srcX; src->y = srcY;
    G.push_back(src);
    
    // Populate the graph
	graph2D::node_t * dst;
    while (!this->addNode(dst));

    // Use A* algorithm to compute the shortest path
	printf("src: %p dst: %p\n", src, dst);
    graph2D::solve(src, dst, G);

    // Store the path taken
    //       Count the number of nodes from dst to src
    cnt = 1;
    for (graph2D::node_t * aux = G.back(); aux->parent != NULL; aux = aux->parent) cnt++;
    //       Allocate memory for steps
    x = (float*)malloc(cnt*sizeof(float));
    y = (float*)malloc(cnt*sizeof(float));
    //       Store step values
    size_t idx = cnt - 1;
    for (graph2D::node_t * aux = G.back(); aux != NULL; aux = aux->parent){
        x[idx] = aux->x;
        y[idx] = aux->y;
        idx--;
    }
    
    // Remove the added edges from the graph
    graph2D::removeNode(src);
    G.pop_back();
}
