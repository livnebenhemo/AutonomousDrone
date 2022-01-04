//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_POLYGON_H
#define ORB_SLAM2_POLYGON_H

#include "Point.h"
#include "Auxiliary.h"
#include "Line.h"
#include "Pizza.h"
#include <set>
#include <vector>
#include <algorithm>
#include "DBSCAN.h"

class Polygon {
public:
    Polygon(std::vector<Point> points, const Point& polygonCenter, bool isExit = false);

    std::vector<Point> getExitPointsByPolygon(bool isDebug = false);

private:
    std::vector<std::pair<Point, double>> getRawPolygonCorners();

    void smoothPolygon(int angleRange = 10);

    void createPointsWithDistance();

    void filterPointsInsidePolygon();

    std::vector<std::pair<double, std::vector<Point>>> getSlicesWithVariances(int currentAngle);

    std::vector<Point> points;

    std::vector<Point> getNavigationPoints(const std::vector<Point>& goodPoints, int minSamples = 15);

    std::vector<Point> filterCheckpoints(const std::vector<Point>& rawNavigationPoints, int minAngleDistance = 20) const;

    Point getNavigationPointFromCluster(const std::vector<Point>& cluster);

    std::vector<Point>
    filterPointsByVariances(const std::vector<std::pair<double, std::vector<Point>>>& slices, double epsilon);

    std::vector<std::pair<Point, double>> pointsWithDistance;
    std::vector<std::pair<Point, double>> pointsOutsidePolygon;
    Point polygonCenter;
    std::vector<Point> vertices;
    std::vector<Line> edges;
    int angle = 25;
    bool isExit;
};


#endif //ORB_SLAM2_POLYGON_H
