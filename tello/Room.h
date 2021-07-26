//
// Created by rbdstudent on 15/06/2021.
//

#ifndef TELLO_ROOM_H
#define TELLO_ROOM_H

#include "Point.h"
#include "Frame.h"
#include <unordered_map>

class Room {
public:
    Room();

    Room(std::vector<Point> points, std::vector<Point> roomCorners, std::vector<Point> exitPoints,
         std::unordered_map<int, Frame> frames);

    std::vector<Point> points;
    std::vector<Point> roomCorners;
    std::vector<Point> visitedExitPoints;
    std::vector<Point> exitPoints;
    std::unordered_map<int, Frame> frames;
};


#endif //TELLO_ROOM_H
