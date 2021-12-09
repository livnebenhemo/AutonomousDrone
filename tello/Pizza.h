//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_PIZZA_H
#define ORB_SLAM2_PIZZA_H

#include "Line.h"
#include <cmath>
#include "Auxiliary.h"
#include <unordered_map>
#include <vector>

class Pizza {
public:
    static std::vector<Line> createPizzaLines(const Point& center, int angle);

    static std::unordered_map<int, std::vector<std::pair<Point, double>>>
    createPizzaSlices(const Point& pizzaCenter, const std::vector<std::pair<Point, double>>& pointsWithDistance, int angle);
};


#endif //ORB_SLAM2_PIZZA_H
