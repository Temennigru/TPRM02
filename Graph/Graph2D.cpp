#include <string>
#include <algorithm>
#include <functional>
#include <vector>
#include <list>
#include <set>
#include <queue>
#include "Graph2D.h"

namespace graph2D{

    typedef node_t* node_t_ptr;

    class nodeGreater : public std::binary_function<std::pair<node_t_ptr, int>, std::pair<node_t_ptr, int>, bool> {
        bool operator() (const std::pair<node_t_ptr, int>& x, const std::pair<node_t_ptr, int>& y) const {return x.first->cost > y.first->cost;;}
    };

    class Frontier {
    public:
        virtual std::pair<node_t_ptr, int> pop () = 0;
        virtual void push(std::pair<node_t_ptr, int>) = 0;
        virtual bool empty() = 0;
    };

    class AStarFrontier : public Frontier {
    private:
        std::priority_queue<
            std::pair<node_t_ptr, int>,
            std::deque<std::pair<node_t_ptr, int> >,
            nodeGreater> frontier;

    public:
        std::pair<node_t_ptr, int> pop () {
            std::pair<node_t_ptr, int> aux = this->frontier.top();
            this->frontier.pop();
            return aux;
        }

        void push(std::pair<node_t_ptr, int> data) {
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
		for each (auto child in adjCopy) removeEdge(n, child);
	}

	// Generic solution template using a frontier
	template<typename frontier_t>void solve(
		node_t * src, // The starting point of the search
		node_t * dst  // The end point of the search
    ){

		// Creates the visitor which does the search
		frontier_t frontier;

        frontier.push(std::make_pair(src, 0));

        while (!frontier.empty) {

            std::pair<node_t_ptr, int> dataPair = frontier.pop();
            node_t_ptr node = dataPair.first;

			// Skips this node if it has already been visited
			if (node->visited){ continue; }

			node->visited = true;

			// Stop the search if this is the solution
			if (node == dst) return;

			// Otherwise, expand the node and add its children to the frontier
			for each (auto child in node->adj){
				double newCost = node->cost + sqdistance(node, child) + sqdistance(child, dst);
                frontier.push(std::make_pair(child, newCost));
                // Update parent and cost in node
                if (child->parent == NULL || newCost < child->cost) {
                    child->cost = newCost;
                    child->parent = parent;
                }
			}
        }

	}

	// Solves the puzzle using A*
	void solve(node_t * src, node_t * dst, std::vector<node_t*> G){

		// Initialize the nodes of the graph
		for each (auto node in G){
			node->parent = NULL;
			node->visited = false;
		}
		src->cost = 0;

		solve<AStarFrontier>(src, dst);
	}
}
