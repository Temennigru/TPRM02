//
//  RRT.cpp
//  TPRM01
//
//  Created by Jean-Luc Nacif Coelho on 9/30/14.
//  Copyright (c) 2014 GrupoRM. All rights reserved.
//

#include "RRT.h"
#include <cstdlib>


#define MAX_DIST 4.0f


// Adds a node to the graph, computing it's visibility towards all other nodes already in the graph
// Return if found dst
bool RRT_t::addNode(graph2D::node_t_ptr dst){

    srand ((int)time(NULL));

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
        node->x = ( ( (node->x - closest_node->x) / graph2D::sqdistance(node, closest_node) ) * MAX_DIST) + closest_node->x;
        node->y = ( ( (node->y - closest_node->y) / graph2D::sqdistance(node, closest_node) ) * MAX_DIST) + closest_node->y;
    }

    // Check visibility to closest node
    // Check a few points allong the path to see if they are occupied or not (heuristic approach)
    bool visible = true;
    bool valid = restrictions->isUnocupied(node->x, node->y);
    const size_t checkedPoints = 1;

    double xInc = ((double)node->x - (double)closest_node->x) / (checkedPoints + 1);
    double yInc = ((double)node->y - (double)closest_node->y) / (checkedPoints + 1);


    for (size_t i = 1; i <= checkedPoints && visible; i++) {
        visible = restrictions->isUnocupied(closest_node->x + xInc*i, closest_node->y + yInc*i);
    }


    // Add the edge and insert the node if there is visibility between the two edges
    if (visible && valid) {
        graph2D::addEdge(closest_node, node);

        // Insert the node into the graph-vector
        G.push_back(node);
    }

    if (graph2D::sqdistance(node, dst) < MAX_DIST) {
        // Check visibility

        visible = true;
        valid = true;

        double xInc = ((double)dst->x - (double)node->x) / (checkedPoints + 1);
        double yInc = ((double)dst->y - (double)node->y) / (checkedPoints + 1);

        for (size_t i = 1; i <= checkedPoints && visible; i++) {
            visible = restrictions->isUnocupied(node->x + xInc*i, node->y + yInc*i);
        }

        if (visible) {
            G.push_back(dst);
            return true;
        }
    }
    return false;
}

// Constructor/Destructor
RRT_t::RRT_t(MovementRestrictions_t * restrictions) :
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
    addNode(src);

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
    for (graph2D::node_t * aux = dst; aux->parent != NULL; aux = aux->parent){
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