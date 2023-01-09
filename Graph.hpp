//including HPP files
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <queue>
#include <limits>
#include <map>

#include "GraphBase.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"

class Graph:public GraphBase
{
    public:             //default functions
        Graph() {} 
        ~Graph(); 

        void addVertex(std::string label); //addVertex
        void removeVertex(std::string label);   //removeVertex
        void addEdge(std::string label1, std::string label2, unsigned long weight);
        void removeEdge(std::string label1, std::string label2);
        unsigned long shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path);

    protected:  // Utility functions
        void singleStr(std::string input_str);                            
        void twoStr(std::string input_str1, std::string input_str2);                          
        void initialPath(std::string startLabel);                             
        void constantPath(std::string startLabel); 
        void minDistance(std::string startLabel);  
        void minIndex(std::string &elementMin);                          
        void pathTravel(std::string endLabel, std::vector<std::string> &path); 
        void freshPath();                                                   

    private:   //Utlility functions
        unsigned long disShort = 0; 
        int vertexMin = 0;               
        std::deque<Vertex *> vrts;    
        std::deque<Edge *> edgs;         
        std::string elementMin = "";   
        std::priority_queue<std::pair<unsigned long, std::string>, std::vector<std::pair<unsigned long, std::string>>, std::greater<std::pair<unsigned long, std::string>>> pathList;
};
#endif 