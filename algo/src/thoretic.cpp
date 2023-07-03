//
// Created by livne on 7/2/23.
//

#include <iostream>
#include "include/thoretic.h"


thoretic::thoretic(std::vector<Point> input_points) {
    this->input_points = input_points;
}


std::vector<Point> thoretic::getOptimalRectangle(std::vector<Point> input_points) {
    std::vector<Point> optimal_rect(4);
    double min_distance_sum = std::numeric_limits<double>::max();

    // Iterate over every combination of 4 points  // TODO : nned to fix it - 5 points : 2 for the first line, and one for each other line
    for (size_t i = 0; i < input_points.size() - 4; ++i) {  // TODO : switch the nested loops with combinations
        for (size_t j = i + 1; j < input_points.size() - 3; ++j) {
            for (size_t k = j + 1; k < input_points.size() - 2; ++k) {
                for (size_t l = k + 1; l < input_points.size()-1; ++l) {
                    for (size_t m = l + 1; m < input_points.size(); ++m) {
                        std::vector<Point> rect = { input_points[i], input_points[j], input_points[k], input_points[l], input_points[m]  };
                        double distance_sum = calculateDistanceSum(input_points, rect);
                        if (distance_sum < min_distance_sum) {
                            min_distance_sum = distance_sum;
                            optimal_rect = rect;
                        }
                    }
                }
            }
        }
    }
    return optimal_rect;
}

double thoretic::calculateDistanceSum(const std::vector<Point>& points, const std::vector<Point>& rect) {
    double sum = 0.0;
    Line line1(rect[0], rect[1]);
    auto slope = line1.slope;
    Line line2(rect[2], slope);
    Line line3(rect[3], -1 / slope);
    Line line4(rect[4], -1 / slope);
    std::vector<Line> rect_lines = {line1, line2, line3, line4};
    for (const Point& point : points) {
        double min_distance = std::numeric_limits<double>::max();
        for (const Line& line : rect_lines) {
            double distance = line.getDistanceToPoint(point);
            min_distance = std::min(min_distance, distance);
        }
        sum += min_distance;
    }
    return sum;
}
