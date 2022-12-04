//
// Created by tzuk on 12/01/2022.
//

#include "include/RRT.h"

RRT::RRT(std::pair<Point, Point> &track, std::vector<Point> &points, bool debug, int numberOfIterations, double radius,
         double stepSize) {
    graph = Graph(track);
    this->numberOfIterations = numberOfIterations;
    this->radius = radius;
    this->stepSize = stepSize;
    this->points = points;
    this->track = track;
    this->debug = debug;
}

/*std::vector<Point> RRT::informedStartSearch() {
    nodes = std::vector<Node>{Node(track.first.x, track.first.y)};
    double bestsCost = std::numeric_limits<double>::max();
    std::set<Node> solutionSet;
    std::vector<Node> path;
    double minCost = Auxiliary::calculateDistanceXY(track.first,track.second);


}*/

Graph RRT::BuildTrack() {
    Navigation navigation;
    if (navigation.objectDetection(points, track, 2, debug)) { // if we have a straight line we don't need pathing
        return graph;
    }
    std::cout << "there is no straight line in the original track" << std::endl;
    std::cout << numberOfIterations << std::endl;
    for (int i = 0; i < numberOfIterations; ++i) {
        auto randomVertex = graph.randomPosition();
        if (Auxiliary::calculateDistance3D(randomVertex, graph.end) > stepSize * 4) {
            i--;
            continue;
        }
        auto nearest = getNearestVertex(randomVertex);
        if (nearest.second == -1) {
            continue;
        }
        std::pair<Point, Point> currentTrack{nearest.first, randomVertex};
        if (navigation.objectDetection(points, currentTrack, 2, debug)) {

            Point distance(randomVertex.x - nearest.first.x, randomVertex.y - nearest.first.y, track.first.z);
            double length = Auxiliary::norm2d(distance.x, distance.y);
            double min = std::min(length, stepSize);
            distance.x = (distance.x / length) * min;
            distance.y = (distance.y / length) * min;
            Point newVertex(nearest.first.x + distance.x, nearest.first.y + distance.y, track.first.z);
            auto newId = graph.addVertex(newVertex);
            graph.addEdge(newId, nearest.second, Auxiliary::calculateDistance3D(newVertex, nearest.first));
            double dist = Auxiliary::calculateDistance3D(newVertex, graph.end);
            if (dist < 2 * radius) {
                graph.addEdge(nearest.second, graph.addVertex(graph.end), dist);
                return graph;
            }
        }
    }
    std::cout << "empty graph" << std::endl;
    return {};
}

std::pair<Point, size_t> RRT::getNearestVertex(Point &newVertex) {
    Point nearVertex;
    size_t nearId = -1;
    double minDist = std::numeric_limits<double>::max();
    for (const auto &vertex: graph.verticesPositions) {
        double currentDistance = Auxiliary::calculateDistanceXY(newVertex, vertex.first);
        if (currentDistance < minDist) {
            minDist = currentDistance;
            nearId = vertex.second;
            nearVertex = vertex.first;
        }
    }
    return {nearVertex, nearId};
}
