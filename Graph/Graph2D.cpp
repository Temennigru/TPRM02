#include <string>
#include <algorithm>
#include <functional>
#include <vector>
#include <list>
#include <set>
#include "Graph2D.h"

namespace graph2D{

	double sqdistance(node_t * a, node_t * b){
		return (a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y);
	}

	void addEdge(node_t * a, node_t * b){
		a->adj.insert(b);
		b->adj.insert(a);
	}

	void removeEdge(node_t * a, node_t * b){
		a->adj.erase(b);
		b->adj.erase(a);
	}

	void removeNode(node_t * n){
		auto adjCopy = n->adj;
		for each (auto child in adjCopy) removeEdge(n, child);
	}

	// Recursively visits the nodes in the graph;
	void recurse(node_t * node, std::function<node_t*(node_t*)> visitor){
		while (node) node = visitor(node);
	}

	// Generic solution template using a frontier
	template<typename frontier_t>void solve(
		node_t * src, // The starting point of the search
		node_t * dst, // The end point of the search
		std::function<void(frontier_t*, node_t *)> insert,  // Inserts an item into the frontier
		std::function<node_t*(frontier_t*)> pop             // Pops an item from the frontier
		){

		// Creates the visitor which does the search
		frontier_t frontier;

		// Visitor that directs tree navigation
		std::function<node_t*(node_t *)> visitor = [&](node_t * node){

			// Skips this node if it has already been visited
			if (node->visited){
				if (frontier.size()) return pop(&frontier);
				else return (node_t*)NULL;
			}
			node->visited = true;

			// Stop the search if this is the solution
			if (node == dst) return (node_t*)NULL;

			// Otherwise, expand the node and add its children to the frontier
			for each (auto child in node->adj){
				double newCost = node->cost + sqdistance(node, child) + sqdistance(child, dst);
				if (newCost < child->cost){
					child->cost = newCost;
					child->parent = node;
					insert(&frontier, child);
				}
			}

			// Return an item from the frontier
			if (frontier.size()) return pop(&frontier);
			else return (node_t*)NULL;
		};

		// Solves by recursively applying the visitor
		recurse(src, visitor);
	}

	// Solves the puzzle using A*
	void solve(node_t * src, node_t * dst, std::vector<node_t*> G){

		// Initialize the nodes of the graph
		for each (auto node in G){
			node->cost = DBL_MAX;
			node->parent = NULL;
			node->visited = false;
		}
		src->cost = 0;

		// Call the generic solver
		std::function<bool(node_t*, node_t*)> comparator =
			[&](node_t * a, node_t * b){ return a->cost > b->cost; };
		std::function<void(std::vector<node_t*>*, node_t *)> insert =
			[&](std::vector<node_t*> * frontier, node_t * child){ frontier->push_back(child); std::push_heap(frontier->begin(), frontier->end(), comparator); };
		std::function<node_t*(std::vector<node_t*>*)> pop =
			[&](std::vector<node_t*> * frontier){ node_t * retval = frontier->front(); std::pop_heap(frontier->begin(), frontier->end(), comparator);  frontier->pop_back(); return retval; };
		solve(src, dst, insert, pop);
	}
}
