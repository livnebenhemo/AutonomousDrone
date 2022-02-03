//
// Created by rbdstudent on 15/06/2021.
//

#include "../include/Frame.h"

#include <utility>
Frame::Frame() {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->rotationMatrix = cv::Mat::zeros(3,3,CV_64F);
    this->frameId = 0;
    this->amountOfKeyPoints = 0;
    this->points = std::vector<Point>{};
}
Frame::Frame(double x, double y, double z,cv::Mat &rotationMatrix, int frameId,
             std::vector<Point> points,int amountOfKeyPoints) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->rotationMatrix = rotationMatrix;
    this->frameId = frameId;
    this->amountOfKeyPoints = amountOfKeyPoints;
    this->points = std::move(points);
}