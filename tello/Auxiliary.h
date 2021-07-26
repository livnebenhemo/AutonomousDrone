//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_AUXILIARY_H
#define ORB_SLAM2_AUXILIARY_H

#include "Point.h"
#include "Line.h"
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Eigen>
//#ifdef NOTRPI
#include <matplotlibcpp.h>

//#endif
class Auxiliary {
public:
    static Point GetCenterOfMass(std::vector<Point> points);

    static double det(Point point1, Point point2);

    static double distanceBetweenPointAndSegment(Point point, Line segment);

    static double angleToRadians(int angle);

    static double getAngleFromSlope(double slope);

    static double radiansToAngle(double radian);

    static double calculateDistance(Point point1, Point point2);
    static double calculateDistance3D(Point point1, Point point2);

    static double getDistanceToClosestSegment(Point point, std::vector<Line> segments);

    static double getAngleBySlopes(Line line1, Line line2);
    static double calculateVariance(std::vector<double> distances);
    static double calculateMeanOfDistanceDifferences(std::vector<double> distances);
    static std::pair<int, bool> getRotationToTargetInFront(Point point1, Point point2);

    static std::pair<int, bool>
    getRotationToTargetInFront(Point previous, Point current, Point destination, bool isMinusUp);

    static std::vector<double> getXValues(std::vector<Point> points);

    static std::vector<double> getYValues(std::vector<Point> points);

//#ifdef NOTRPI
    static void showCloudPoint(std::vector<Point> redPoints, std::vector<Point> cloud);
//#endif
};


#endif //ORB_SLAM2_AUXILIARY_H
