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

    Frame(double x, double y, double z, cv::Mat &rotationMatrix, int frameId,
          std::vector<Point> points, int amountOfKeyPoints);

    Frame &operator=(const Frame &frame) {
        this->x = frame.x;
        this->y = frame.y;
        this->z = frame.z;
        this->rotationMatrix = frame.rotationMatrix;
        this->points = frame.points;
        this->frameId = frame.frameId;
        this->amountOfKeyPoints = frame.amountOfKeyPoints;
        return *this;
    }

    double x;
    double y;
    double z;
    cv::Mat rotationMatrix;
    int frameId;
    int amountOfKeyPoints;
    std::vector<Point> points;
};


#endif //TELLO_FRAME_H
