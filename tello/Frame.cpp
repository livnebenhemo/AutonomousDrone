//
// Created by rbdstudent on 15/06/2021.
//

#include "Frame.h"
Frame::Frame() {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->qx = 0;
    this->qy = 0;
    this->qz = 0;
    this->qw = 0;
    this->frameId = 0;
    this->amountOfKeyPoints = 0;
    this->points = std::vector<Point>{};
}
Frame::Frame(double x, double y, double z, double qx, double qy, double qz, double qw, int frameId,
             std::vector<Point> points,int amountOfKeyPoints) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->qx = qx;
    this->qy = qy;
    this->qz = qz;
    this->qw = qw;
    this->frameId = frameId;
    this->amountOfKeyPoints = amountOfKeyPoints;
    this->points = points;
}