//including HPP files

#ifndef EDGE_HPP
#define EDGE_HPP

#include <string>

class Edge
{
    public:
        friend class Graph; //making a friend class to give special access

   
        Edge(std::string a = "", std::string b = "", unsigned long w = 0)
        {
            currEdge = a;
            adjEdge = b;
            weightEdge = w;
        }
        ~Edge() {}

    private:
        std::string currEdge;  // Current edge
        std::string adjEdge;   // Adjacent edge
        unsigned long weightEdge; 
};
#endif 