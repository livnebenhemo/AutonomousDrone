//
// Created by rbdstudent on 15/06/2021.
//

#include "tello/include/Room.h"

#include <utility>

Room::Room(std::vector<Point> points, std::vector<Point> roomCorners, std::vector<Point> exitPoints,
           std::unordered_map<int, Frame> frames) {
    this->points = std::move(points);
    this->exitPoints = std::move(exitPoints);
    this->roomCorners = std::move(roomCorners);
    this->frames = std::move(frames);
}

Room::Room() {
    points = std::vector<Point>{};
    exitPoints = std::vector<Point>{};
    roomCorners = std::vector<Point>{};
    frames = std::unordered_map<int, Frame>{};
}