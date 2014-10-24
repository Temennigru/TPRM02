#include <string>
#include <algorithm>
#include <functional>
#include <vector>
#include <list>
#include <set>
#include <queue>
#include <iostream>
#include "Graph2D.h"

namespace graph2D{

    class nodeGreater : public std::binary_function<std::pair<node_t_ptr, nodeValue_t>, std::pair<node_t_ptr, nodeValue_t>, bool> {
    public:
        bool operator() (const std::pair<node_t_ptr, nodeValue_t>& x, const std::pair<node_t_ptr, nodeValue_t>& y) const {return x.first->cost > y.first->cost;;}
    };

    class Frontier {
    public:
        virtual std::pair<node_t_ptr, double> pop () = 0;
        virtual void push(std::pair<node_t_ptr, double>) = 0;
        virtual bool empty() = 0;
    };

    class AStarFrontier : public Frontier {
    private:
        std::priority_queue<
            std::pair<node_t_ptr, double>,
            std::deque<std::pair<node_t_ptr, double> >,
            nodeGreater> frontier;

    public:
        std::pair<node_t_ptr, double> pop () {
            std::pair<node_t_ptr, double> aux = this->frontier.top();
            this->frontier.pop();
            return aux;
        }

        void push(std::pair<node_t_ptr, double> data) {
            this->frontier.push(data);
        }

        bool empty() {
            return this->frontier.empty();
        }
    };

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
		for (auto child = adjCopy.begin(); child != adjCopy.end(); child++) removeEdge(n, *child);
	}

	// Generic solution template using a frontier
	template<typename frontier_t>void solve(
		node_t * src, // The starting point of the search
		node_t * dst  // The end point of the search
    ){

		// Creates the visitor which does the search
		frontier_t frontier;

        frontier.push(std::make_pair(src, 0));

        while (!frontier.empty()) {

            std::pair<node_t_ptr, nodeValue_t> dataPair = frontier.pop();
            node_t_ptr node = dataPair.first;

			// Skips this node if it has already been visited
			if (node->visited){ continue; }

			node->visited = true;

			// Stop the search if this is the solution
			if (node == dst) return;

			// Otherwise, expand the node and add its children to the frontier
            for (auto child = node->adj.begin(); child != node->adj.end(); child++) {
                double newCost = node->cost + sqdistance(node, *child) + sqdistance(*child, dst);
                frontier.push(std::make_pair(*child, newCost));
                // Update parent and cost in node
                if (((*child)->parent == NULL || newCost < (*child)->cost) && *child != src) {
                    (*child)->cost = newCost;
                    (*child)->parent = node;
                }
            }
        }

	}

	// Solves the puzzle using A*
	void solve(node_t * src, node_t * dst, std::vector<node_t*> G){

		// Initialize the nodes of the graph
		for (auto node = G.begin(); node != G.end(); node++) {
			(*node)->parent = NULL;
			(*node)->visited = false;
        }
		src->cost = 0;

		solve<AStarFrontier>(src, dst);
	}
}
