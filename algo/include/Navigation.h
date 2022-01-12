//
// Created by tzuk on 02/01/2022.
//

#ifndef ORB_SLAM2_NAVIGATION_H
#define ORB_SLAM2_NAVIGATION_H

#include "include/Point.h"
#include <vector>
#include "DBSCAN.h"
#include "../utils/include/Frame.h"

class Navigation {
public:
    std::vector<Point>
    getValidNavigationPointsByDBScan(const std::vector<Frame> &frames, const std::vector<Point> &points,
                                     const Point &currentLocation);

    std::vector<Point> getFloor(std::vector<Point> &points, unsigned long sizeOfJump);

    bool
    objectDetection(std::vector<Point> &points, std::pair<Point,Point> &track, bool debug = false);

};


#endif //ORB_SLAM2_NAVIGATION_H
