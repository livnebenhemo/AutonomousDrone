//
// Created by rbdstudent on 15/06/2021.
//

#include "Point.h"


Point::Point() {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->qx = 0;
    this->qy = 0;
    this->qz = 0;
    this->qw = 0;
    this->label = -1;
    this->frameId = 0;
}

Point::Point(const Point &point) {
    this->x = point.x;
    this->y = point.y;
    this->z = point.z;
    this->qx = point.qx;
    this->qy = point.qy;
    this->qz = point.qz;
    this->qw = point.qw;
    this->label = point.label;
    this->frameId = point.frameId;
}

Point::Point(double x, double y, double z, double qx, double qy, double qz, double qw, int frameId, int label) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->qx = qx;
    this->qy = qy;
    this->qz = qz;
    this->qw = qw;
    this->label = label;
    this->frameId = frameId;
}
