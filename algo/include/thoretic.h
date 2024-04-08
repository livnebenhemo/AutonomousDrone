//
// Created by livne on 7/2/23.
//

#ifndef ORB_SLAM2_THORETIC_H
#define ORB_SLAM2_THORETIC_H

#include "include/Point.h"
#include "include/Line.h"
#include "Pizza.h"
#include <vector>
#include <cmath>
#include "DBSCAN.h"

class thoretic {
public:
    thoretic(std::vector<Point> input_points);
    std::vector<Point> input_points;

    static std::vector<Point> getOptimalRectangle(std::vector<Point> input_points, const std::vector<int>& weights);
    static std::vector<Point> getVerticesOfRectangle(std::vector<Point> rect);
    static Point findIntersection(const Line& line1, const Line& line2);

    static std::vector<Point> getExitPointsByRectangle(std::vector<Point> rect_vertices, std::vector<Point> points, const Point &currentPosition, bool isDebug = false, int angle = 10);
    static std::vector<Point> filterPointsInsideRectangle(std::vector<Point> vertices, std::vector<Point> points);
    static std::vector<Point> filterCheckpoints(std::vector<Point> navigationPoints, const Point &currentPosition, int minAngleDistance = 20);
    static std::vector<Point> getNavigationPoints(std::vector<Point> goodPoints, int minSamples=15); // TODO : defince default variables
    static Point getNavigationPointFromCluster(std::vector<Point> cluster);
    static std::vector<std::pair<Point, double>> createPointsWithDistanceFromCurrentPosition(std::vector<Point> points, const Point &currentPosition);
    static std::vector<std::pair<double, std::vector<Point>>> getSlicesWithVariances(const std::vector<std::pair<Point, double>> pointsOutsidePolygonWithDistance, const Point &currentPosition, int angle);
    static std::vector<Point> filterPointsByVariances(std::vector<std::pair<double, std::vector<Point>>> slices, std::vector<Point> rect, double epsilon);
    static double calculateDistanceSum(const std::vector<Point>& points, const std::vector<Point>& rect, const std::vector<int>& weights);
    static std::vector<double> normalize_vector(const std::vector<int>& input_vector);
};


#endif //ORB_SLAM2_THORETIC_H
