//
// Created by tzuk on 02/01/2022.
//

#ifndef ORB_SLAM2_NAVIGATION_H
#define ORB_SLAM2_NAVIGATION_H

#include "include/Point.h"
#include <vector>
#include "include/DBSCAN.h"
#include "tello/include/Frame.h"

class Navigation {
public:
    std::vector<Point> getValidNavigationPoints(const std::vector<Frame> &frames, const std::vector<Point> &points,
                                                const Point &currentLocation);
};


#endif //ORB_SLAM2_NAVIGATION_H
