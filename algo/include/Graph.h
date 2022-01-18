//
// Created by tzuk on 12/01/2022.
//

#ifndef ORB_SLAM2_GRAPH_H
#define ORB_SLAM2_GRAPH_H


#include <utility>
#include <vector>
#include <unordered_map>
#include "include/Point.h"

class Graph {

    std::vector<std::pair<size_t, size_t>> edges;
    std::unordered_map<size_t, double> distances;
    double sx;
    double sy;
    double maxX;
    double minX;
    double maxY;
    double minY;


public:
    Graph();

    explicit Graph(std::pair<Point, Point> &track);

    std::vector<Point> vertices;
    std::unordered_map<Point, size_t> verticesPositions;
    std::unordered_map<size_t, std::vector<std::pair<size_t, double>>> neighbors;

    size_t addVertex(const Point& pos);

    void addEdge(size_t id1, size_t id2, double distance);
    Point randomPosition() const;

    Point end;
    Point start;
};


#endif //ORB_SLAM2_GRAPH_H
