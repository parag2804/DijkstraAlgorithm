//including HPP files

#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <string>
#include <vector>

class Vertex
{
    public:
        friend class Graph; //making a friend class to give special access
        
        Vertex() // Defining constructor and destructor
        {
            vertex = "";
        }

        ~Vertex() {}

    private:
        std::string vertex;                  // Vertex node
        unsigned long distMin;               // Minimum distance
        std::vector<std::string> pathShort; //  Shortest paths in the vector
        bool node;                          
};
#endif