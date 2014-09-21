//
//  Graph.h
//  TPRM01
//
//  Created by Jean-Luc Nacif Coelho on 9/19/14.
//  Copyright (c) 2014 GrupoRM. All rights reserved.
//

#ifndef TPRM01_Graph_h
#define TPRM01_Graph_h

class Node {
    private:
    int m_x, m_y;
    int nodeId;
    std::vector<Node> nxt;

    public:
    int X(); // Get X
    int Y(); // Get Y
    void X(int x); // Set X
    void Y(int y); // Set Y

    const std::vector<Node>& getNxt();
    int getId();

    Node(int x, int y);

    friend class Graph;

};

class Graph {
    private:
    std::vector<Node> nodes;

    public:

    void insert(int x, int y);
    void insert(int x, int y);
    void connect(Node node1, Node node2);

    const std::vector<Node>& getNodes();

    const std::vector<Node> shortestPath(int id1, int id2); // Returns a vector with the path to take in orders

    bool isVisible (Node node1, Node node2, const Graph& obstacle);


};

class GraphTraversal


#endif
