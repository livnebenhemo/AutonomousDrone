//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_LINE_H
#define ORB_SLAM2_LINE_H

#include "Point.h"
#include <cmath>

class Line {
public:
    Line(Point point1, Point point2);

    Line(Point point, double slope);
    double getSlope(){
        return slope;
    }
    Point getPoint2(){
        return point2;
    }
    Point getPoint1(){
        return point1;
    }
    double getDistanceToPoint(Point point);
    double getDistanceToSegment(Point point);

    Point getLineIntersection(Line line);

private:
    Point point1;
    Point point2;
    double slope;
    double yIntercept;
};


#endif //ORB_SLAM2_LINE_H
