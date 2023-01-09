//Description: In this project we will use Dijkstra's algorithm to find the shortest path between two verteces by implementing an undirected weighted Graph ADT

#include "Graph.hpp"

Graph::~Graph()         //Destructor to delete
{
    for (auto &a: vrts)
        delete a;

    for (auto &a: edgs)
        delete a;

    vrts.clear();
    edgs.clear();
}

void Graph::addVertex(std::string label)        //function to add vertex using push
{
    singleStr(label);
    Vertex *vrtNew = new Vertex;
    vrtNew->vertex = label;
    vrtNew->distMin = std::numeric_limits<unsigned long>::max();
    vrtNew->pathShort = {""};
    vrtNew->node = false;
    vrts.push_back(vrtNew);
}

void Graph::removeVertex(std::string label) //function to remove vertex
{
    singleStr(label);

    for(std::deque<Vertex *>::iterator ver_itr = vrts.begin(); ver_itr != vrts.end();)
    {
        if ((*ver_itr)->vertex == label)
            vrts.erase(ver_itr);

        ++ver_itr;
    }

    for (std::deque<Edge *>::iterator edg_itr = edgs.begin(); edg_itr != edgs.end();) //loop to remove edge
    {
        if (((*edg_itr)->adjEdge == label) || ((*edg_itr)->currEdge == label))
            edgs.erase(edg_itr);
        ++edg_itr;
    }
}

void Graph::addEdge(std::string label1, std::string label2, unsigned long weightEdge)
{
    twoStr(label1, label2);// Adds edge between the vertex with label1 and label2.
    Edge *edgeOne = new Edge(label1, label2, weightEdge);
    Edge *edgeTwo = new Edge(label2, label1, weightEdge);

    edgs.push_back(edgeOne);
    edgs.push_back(edgeTwo);
}
void Graph::removeEdge(std::string label1, std::string label2) // Removes edge between the vertex with label1 and label2.
{
    twoStr(label1, label2);
    for (std::deque<Edge *>::iterator edg_itr = edgs.begin(); edg_itr != edgs.end();)
    {
        if (((*edg_itr)->currEdge == label1) && ((*edg_itr)->adjEdge == label2))
            edgs.erase(edg_itr);

        ++edg_itr;
    }
}

void Graph::singleStr(std::string input_str) //Checking for NULL
{
    if (input_str.empty())
        throw input_str;
}

void Graph::twoStr(std::string input_str1, std::string input_str2) //Checking for NULL
{
    if (input_str1.empty())
        throw input_str1;

    else if (input_str2.empty())
        throw input_str2;

    else if (input_str1.empty() && input_str2.empty())
        throw("String arguments are empty.\n");
}


unsigned long Graph::shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path)
//Calculates the shortest path between startLabel and endLabel using Dijkstra's Algorithm.
{
    twoStr(startLabel, endLabel);
    initialPath(startLabel);
    constantPath(startLabel);
    pathTravel(endLabel, path);
    freshPath();

    return disShort;
    // Return the shortest distance
}

void Graph::initialPath(std::string startLabel)
{
    for(std::deque<Vertex *>::iterator ver_itr = vrts.begin(); ver_itr != vrts.end();)
    {
        if ((*ver_itr)->vertex == startLabel)
        {
            (*ver_itr)->distMin = 0;
            (*ver_itr)->pathShort.clear();
            (*ver_itr)->pathShort.push_back(startLabel);
        }
        ++ver_itr;
    }
    pathList.push(std::make_pair(0, startLabel));
}

void Graph::minIndex(std::string &elementMin)
{
    int ver_idx = 0;
    for (std::deque<Vertex *>::iterator ver_itr = vrts.begin(); ver_itr != vrts.end();) //traverse the vector
    {
        if ((*ver_itr)->vertex == elementMin)
            vertexMin = ver_idx;

        else
            ++ver_idx;

        ++ver_itr;
    }
}

void Graph::minDistance(std::string startLabel) // traverse through edge and vertex containers using bfs
{

    for (std::deque<Edge *>::iterator edg_itr = edgs.begin(); edg_itr != edgs.end();)
    {
        if ((*edg_itr)->currEdge == elementMin)
        {
            for (std::deque<Vertex *>::iterator ver_itr = vrts.begin(); ver_itr != vrts.end();)
            {
                if (((*edg_itr)->adjEdge == (*ver_itr)->vertex) && ((vrts.at(vertexMin)->distMin + (*edg_itr)->weightEdge) < (*ver_itr)->distMin) && ((*ver_itr)->node == false))
                {
                    (*ver_itr)->distMin = vrts.at(vertexMin)->distMin + (*edg_itr)->weightEdge;

                    if(elementMin == startLabel)
                    {
                        (*ver_itr)->pathShort.clear();
                        (*ver_itr)->pathShort.push_back(startLabel);
                    }
                    else if(elementMin != startLabel)
                    {
                        (*ver_itr)->pathShort.clear();
                        (*ver_itr)->pathShort = vrts.at(vertexMin)->pathShort;
                        (*ver_itr)->pathShort.push_back(vrts.at(vertexMin)->vertex);
                    }
                    pathList.push(std::make_pair((*ver_itr)->distMin, (*ver_itr)->vertex));
                }
                ++ver_itr;
            }
        }
        ++edg_itr;
    }
}

void Graph::constantPath(std::string startLabel) //traverse from A to B
{
    for (; !pathList.empty();)
    {
        elementMin = pathList.top().second;
        pathList.pop();
        minIndex(elementMin);
        minDistance(startLabel);
        vrts.at(vertexMin)->node = true;
    }
}

void Graph::pathTravel(std::string endLabel, std::vector<std::string> &path) //get shortest distances from A->B
{
    for(std::deque<Vertex *>::iterator ver_itr = vrts.begin(); ver_itr != vrts.end();)
    {
        if(endLabel == (*ver_itr)->vertex)
        {
            disShort = (*ver_itr)->distMin;
            (*ver_itr)->pathShort.push_back(endLabel);
            path = (*ver_itr)->pathShort;
        }
        ++ver_itr;
    }
}
void Graph::freshPath()
{

    for (std::deque<Vertex *>::iterator ver_itr = vrts.begin(); ver_itr != vrts.end();)
    {
        (*ver_itr)->distMin = std::numeric_limits<unsigned long>::max();
        (*ver_itr)->pathShort = {""};
        (*ver_itr)->node = false;
        ++ver_itr;
    }
}
