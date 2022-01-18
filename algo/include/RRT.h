//
// Created by tzuk on 12/01/2022.
//

#ifndef RRT_H
#define RRT_H


#include <utility>
#include "include/Point.h"
#include "Graph.h"
#include "Navigation.h"
#include "include/Node.h"

class RRT {
public:
    RRT(std::pair<Point, Point> &track, std::vector<Point> &points, bool debug = false, int numberOfIterations = 200,
        double radius = 0.3,
        double stepSize = 0.4);

    Graph BuildTrack();

private:
    Graph graph;
    double radius;
    int numberOfIterations;
    double stepSize;
    std::pair<Point, Point> track;
    std::vector<Point> points;
    std::vector<Node> nodes;
    bool debug;

    std::pair<Point, size_t> getNearestVertex(Point &newVertex);
};


#endif //RRT_H
