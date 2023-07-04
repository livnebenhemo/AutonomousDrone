//
// Created by livne on 7/2/23.
//

#ifndef ORB_SLAM2_THORETIC_H
#define ORB_SLAM2_THORETIC_H

#include "include/Point.h"
#include "include/Line.h"
#include <vector>
#include <cmath>

class thoretic {
public:
    thoretic(std::vector<Point> input_points);
    std::vector<Point> input_points;

    static std::vector<Point> getOptimalRectangle(std::vector<Point> input_points);
    static std::vector<Point> getVerticesOfRectangle(std::vector<Point> rect);
    static Point findIntersection(const Line& line1, const Line& line2);

private:
    static double calculateDistanceSum(const std::vector<Point>& points, const std::vector<Point>& rect);
};


#endif //ORB_SLAM2_THORETIC_H
