//
// Created by rbdstudent on 15/06/2021.
//

#ifndef TELLO_FRAME_H
#define TELLO_FRAME_H

#include "Point.h"
#include <vector>

class Frame {
public:
    Frame();
    Frame(double x, double y, double z, double qx, double qy, double qz, double qw, int frameId,
          std::vector<Point> points,int amountOfKeyPoints);
    Frame& operator = (const Frame& frame){
        this->x = frame.x;
        this->y = frame.y;
        this->z = frame.z;
        this->qx = frame.qx;
        this->qy = frame.qy;
        this->qz = frame.qz;
        this->qw = frame.qw;
        this->points = frame.points;
        this->frameId = frame.frameId;
        this->amountOfKeyPoints = frame.amountOfKeyPoints;
        return *this;
    }
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
    int frameId;
    int amountOfKeyPoints;
    std::vector<Point> points;
};


#endif //TELLO_FRAME_H
