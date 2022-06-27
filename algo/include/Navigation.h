//
// Created by tzuk on 02/01/2022.
//

#ifndef ORB_SLAM2_NAVIGATION_H
#define ORB_SLAM2_NAVIGATION_H

#include "include/Point.h"
#include <vector>
#include "DBSCAN.h"
#include "../utils/include/Frame.h"
#include "Graph.h"

class Navigation {
public:
    std::vector<Point> getNavigationPathByRRT(std::vector<Point> &cloud, std::pair<Point,Point> &track, bool debug = false);

    static std::vector<Point> getFloor(std::vector<Point> &points, unsigned long sizeOfJump);

    bool
    objectDetection(std::vector<Point> &points, std::pair<Point,Point> &track,int sizeOfJump = 2, bool debug = false);

    std::vector<Point> filterPointsByStartPosition(std::vector<Point> &points, std::pair<Point, Point> &track);

    std::vector<Point> dijkstra(Graph graph);
};


#endif //ORB_SLAM2_NAVIGATION_H
