//
// Created by livne on 7/2/23.
//

#include "include/thoretic.h"


thoretic::thoretic(std::vector<Point> input_points) {
    this->input_points = input_points;
}


std::vector<Point> thoretic::getOptimalRectangle(std::vector<Point> input_points) {
    std::vector<Point> optimal_rect(4);
    double min_distance_sum = std::numeric_limits<double>::max();

    // Iterate over every combination of 4 points  // TODO : nned to fix it - 5 points : 2 for the first line, and one for each other line
    for (size_t i = 0; i < input_points.size() - 3; ++i) {  // TODO : switch the nested loops with combinations
        for (size_t j = i + 1; j < input_points.size() - 2; ++j) {
            for (size_t k = j + 1; k < input_points.size() - 1; ++k) {
                for (size_t l = k + 1; l < input_points.size(); ++l) {
                    std::vector<Point> rect = { input_points[i], input_points[j], input_points[k], input_points[l] };
                    double distance_sum = calculateDistanceSum(input_points, rect);
                    if (distance_sum < min_distance_sum) {
                        min_distance_sum = distance_sum;
                        optimal_rect = rect;
                    }
                }
            }
        }
    }

    return optimal_rect;
}

double thoretic::calculateDistance(const Point& p, const Point& p1, const Point& p2) {
    // Calculate the distance between point p and the line segment defined by points p1 and p2
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dot = (p.x - p1.x) * dx + (p.y - p1.y) * dy;
    double lengthSquared = dx * dx + dy * dy;
    double t = std::max(0.0, std::min(1.0, dot / lengthSquared));
    double closestX = p1.x + t * dx;
    double closestY = p1.y + t * dy;
    double distance = std::sqrt((p.x - closestX) * (p.x - closestX) + (p.y - closestY) * (p.y - closestY));
    return distance;
}

double thoretic::calculateDistanceSum(const std::vector<Point>& points, const std::vector<Point>& rect) {
    double sum = 0.0;
    for (const Point& point : points) {
        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < rect.size(); ++i) {
            const Point& p1 = rect[i];
            const Point& p2 = rect[(i + 1) % rect.size()];
            double distance = calculateDistance(point, p1, p2);
            min_distance = std::min(min_distance, distance);
        }
        sum += min_distance;
    }
    return sum;
}
