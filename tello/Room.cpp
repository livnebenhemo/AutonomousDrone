//
// Created by rbdstudent on 15/06/2021.
//

#include "Room.h"

Room::Room(std::vector<Point> points, std::vector<Point> roomCorners, std::vector<Point> exitPoints,
           std::unordered_map<int, Frame> frames) {
    this->points = points;
    this->exitPoints = exitPoints;
    this->roomCorners = roomCorners;
    this->frames = frames;
}

Room::Room() {
    points = std::vector<Point>{};
    exitPoints = std::vector<Point>{};
    roomCorners = std::vector<Point>{};
    frames = std::unordered_map<int, Frame>{};
}