#ifndef _GRAPH2D_H_
#define _GRAPH2D_H_

#include <string>
#include <algorithm>
#include <functional>
#include <vector>
#include <list>
#include <set>

namespace graph2D{

	// A node in the graph
	struct node_t {
		
		// Node properties
		size_t x, y;
		std::set<node_t*> adj;

		// Search auxiliaries
		node_t * parent;
		double cost;
		bool visited;
	};

	// Computes the square of the distance between two nodes
	double sqdistance(node_t * a, node_t * b);

	// Adds/removes edges and nodes
	void addEdge(node_t * a, node_t * b);
	void removeEdge(node_t * a, node_t * b);
	void removeNode(node_t * n);

	// Recursively visits the nodes in the graph;
	void recurse(node_t * node, std::function<node_t*(node_t*)> visitor);

	// Solves the puzzle using A*
	void solve(node_t * src, node_t * dst, std::vector<node_t*> G);
}

#endif
