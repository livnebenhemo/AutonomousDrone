//
// Created by tzuk on 12/01/2022.
//

#include <random>
#include <iostream>
#include "include/Graph.h"

Graph::Graph() {
    start = Point();
    end = Point(1, 1, 1);
    vertices = std::vector<Point>{start};
    edges = std::vector<std::pair<size_t, size_t>>{};
    verticesPositions.insert({start, 0});
    distances.insert({0, 0});
    neighbors.insert({0, std::vector<std::pair<size_t, double>>{}});
    sx = start.x - end.x;
    sy = start.y - end.y;
    minX = start.x;
    maxX = end.x;
    minY = start.y;
    maxY = end.y;
}

Graph::Graph(std::pair<Point, Point> &track) {
    start = track.first;
    end = track.second;
    vertices = std::vector<Point>{start};
    edges = std::vector<std::pair<size_t, size_t>>{};
    verticesPositions.insert({start, 0});
    distances.insert({0, 0});
    neighbors.insert({0, std::vector<std::pair<size_t, double>>{}});
    sx = start.x - end.x;
    sy = start.y - end.y;
    minX = start.x*1.5;
    maxX = end.x*1.5;
    minY = start.y*1.5;
    maxY = end.y*1.5;
}

size_t Graph::addVertex(const Point &pos) {
    if (!verticesPositions.count(pos)) {
        size_t newId = verticesPositions.size();
        vertices.emplace_back(pos);
        verticesPositions.insert({pos, newId});
        neighbors.insert({newId, std::vector<std::pair<size_t, double>>{}});
        return newId;
    }
    return -1;
}

void Graph::addEdge(size_t id1, size_t id2, double distance) {
    edges.emplace_back(id1, id2);
    neighbors.at(id1).emplace_back(id2, distance);
    neighbors.at(id2).emplace_back(id1, distance);
}

Point Graph::randomPosition() const {
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> disX(minX, maxX);
    std::uniform_real_distribution<> disY(minY, maxY);
    double randomX = disX(gen);
    double randomY = disY(gen);
    return {(start.x - (sx / 2) + randomX * sx * 2), (start.y - (sy / 2) + randomY * sy * 2), start.z};
}