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
bool RRT_t::addNode(graph2D::node_t_ptr dst){

    graph2D::node_t_ptr node = new graph2D::node_t();
    node->x = (size_t)rand() % restrictions->getWidth();
    node->y = (size_t)rand() % restrictions->getHeight();

    double min_dist = 0.0;
    graph2D::node_t_ptr closest_node = NULL;

    // Find closest node
    for (auto n = G.begin(); n != G.end(); n++) {
        if (closest_node == NULL || graph2D::sqdistance(*n, node) < min_dist) {
            closest_node = *n;
            min_dist = graph2D::sqdistance(*n, node);
        }
    };

    // Fix distance
    if (min_dist > MAX_DIST) {
        node->x = (size_t)( ( ((int)node->x - (int)closest_node->x) / sqrt(graph2D::sqdistance(node, closest_node))) * MAX_DIST) + closest_node->x;
        node->y = (size_t)( ( ((int)node->y - (int)closest_node->y) / sqrt(graph2D::sqdistance(node, closest_node))) * MAX_DIST) + closest_node->y;
    }

    // Check visibility to closest node
    bool visible = restrictions->isUnobstructed(node->x, node->y, closest_node->x, closest_node->y);

    // Add the edge and insert the node if there is visibility between the two edges
    if (visible) {
        graph2D::addEdge(closest_node, node);

        // Insert the node into the graph-vector
        G.push_back(node);
    }

    if (graph2D::sqdistance(node, dst) <= SQ_MAX_DIST) {
        // Check visibility
		graph2D::addEdge(node, dst);		
		G.push_back(dst);
		return true;
    }
    return false;
}

// Constructor/Destructor
RRT_t::RRT_t(MovementRestrictions_t * restrictions) : restrictions(restrictions) {}

RRT_t::~RRT_t(void){
    for (auto node = G.begin(); node != G.end(); node++) delete *node;
}

// Finds a path between two points using the visibility graph
void RRT_t::findPath(
                                 uint16_t srcX, uint16_t srcY, uint16_t dstX, uint16_t dstY,
                                 uint16_t * &x, uint16_t * &y, uint16_t &cnt
                                 ){
    // Add the source and destination as nodes in the graph
    // Dist won't be added as to not mess up the algorithm
    graph2D::node_t * src = new graph2D::node_t(); src->x = srcX; src->y = srcY;
    graph2D::node_t * dst = new graph2D::node_t(); dst->x = dstX; dst->y = dstY;
    G.push_back(src);
    srand((int)time(NULL));
	
    // Populate the graph
    while (!this->addNode(dst)) {}

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
}
