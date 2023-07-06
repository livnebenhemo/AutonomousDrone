//
// Created by livne on 7/2/23.
//

#include <iostream>
#include "include/thoretic.h"
#include "include/Auxiliary.h"


thoretic::thoretic(std::vector<Point> input_points) {
    this->input_points = input_points;
}


Point thoretic::findIntersection(const Line& line1, const Line& line2)
{
    // Check if the lines are parallel
    /*if (line1.slope == line2.slope) {
        return false;
    }*/

    // Calculate the intersection point
    double x = (line2.yIntercept - line1.yIntercept) / (line1.slope - line2.slope);
    double y = line1.slope * x + line1.yIntercept;

    return Point(x, y, 0);
}



std::vector<Point> thoretic::getVerticesOfRectangle(std::vector<Point> rect) {
    Line line1(rect[0], rect[1]);
    auto slope = line1.slope;
    Line line2(rect[2], slope);
    Line line3(rect[3], -1 / slope);
    Line line4(rect[4], -1 / slope);

    Point vertex1 = findIntersection(line1, line3);
    Point vertex2 = findIntersection(line1, line4);
    Point vertex3 = findIntersection(line2, line3);
    Point vertex4 = findIntersection(line2, line4);

    std::vector<Point> vertices = {vertex1, vertex3, vertex4, vertex2};
    return vertices;
}


/*std::vector<Point> thoretic::getOptimalRectangle(std::vector<Point> input_points) {
    std::vector<Point> optimal_rect(5);
    double min_distance_sum = std::numeric_limits<double>::max();

    // Iterate over every combination of 4 points  //
    for (size_t i = 0; i < input_points.size() - 4; ++i) {
        for (size_t j = i + 1; j < input_points.size() - 3; ++j) {
            for (size_t k = j + 1; k < input_points.size() - 2; ++k) {
                for (size_t l = k + 1; l < input_points.size()-1; ++l) {
                    for (size_t m = l + 1; m < input_points.size(); ++m) {
                        std::vector<unsigned long> indices = {i, j, k, l, m};
                        do{
                            std::vector<Point> rect = {input_points[indices[0]], input_points[indices[1]], input_points[indices[2]], input_points[indices[3]], input_points[indices[4]]};
                            double distance_sum = calculateDistanceSum(input_points, rect);
                            if (distance_sum < min_distance_sum) {
                                min_distance_sum = distance_sum;
                                optimal_rect = rect;
                            }
                        } while(std::next_permutation(indices.begin(), indices.end()));
                    }
                }
            }
        }
    }
    return optimal_rect;
}*/


std::vector<Point> thoretic::getOptimalRectangle(std::vector<Point> points) {
    std::vector<Point> optimal_rect(5);
    double min_distance_sum = std::numeric_limits<double>::max();
    auto input_points(points);
    // Random number generator
    std::random_device rd;
    std::mt19937 g(rd());
    // Shuffle
    std::shuffle(input_points.begin(), input_points.end(), g);
    // Iterate over every combination of 4 points  // TODO : nned to fix it - 5 points : 2 for the first line, and one for each other line
    for (size_t i = 0; i < input_points.size() - 4; ++i) {  // TODO : switch the nested loops with combinations
        for (size_t j = i + 1; j < input_points.size()- 3; ++j) {
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
    std::cout << "cost : " << min_distance_sum << std::endl;
    return optimal_rect;
}


/*std::vector<Point> thoretic::getOptimalRectangle(std::vector<Point> input_points) {
    std::vector<Point> optimal_rect(5);
    double min_distance_sum = std::numeric_limits<double>::max();
    // Iterate over every combination of 4 points  // TODO : nned to fix it - 5 points : 2 for the first line, and one for each other line
    long count = 0;
    auto g = std::mt19937 {std::random_device{}()};
    while (count < 5000000) {
        std::vector<Point> rect;
        std::sample(input_points.begin(), input_points.end(), std::back_inserter(rect), 5, g);
        // std::vector<Point> rect = {input_points[i], input_points[j], input_points[k], input_points[l], input_points[m]};
        double distance_sum = calculateDistanceSum(input_points, rect);
        if (distance_sum < min_distance_sum) {
            min_distance_sum = distance_sum;
            optimal_rect = rect;
        }
        count++;
    }
    std::cout << "cost : " << min_distance_sum << std::endl;
    return optimal_rect;
}*/


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


std::vector<Point> thoretic::getExitPointsByRectangle(std::vector<Point> rect_vertices, std::vector<Point> points, const Point &currentPosition, bool isDebug, int angle) {
    // epsilon is a scale measure
    double epsilon = 0.0;
    for (auto vertex : rect_vertices) {
        epsilon += Auxiliary::calculateDistanceXY(currentPosition, vertex);
    }
    epsilon /= rect_vertices.size() * 2;  // TODO : can improve it

    if (isDebug) {
        Auxiliary::showCloudPoint(rect_vertices, points);
    }

    // filter points inside and outside the rectangle
    auto pointsOutsideRect = filterPointsInsideRectangle(rect_vertices, points);
    auto outside_point_with_distance_from_current_position = createPointsWithDistanceFromCurrentPosition(pointsOutsideRect, currentPosition);
    auto slices_with_variance = getSlicesWithVariances(outside_point_with_distance_from_current_position, currentPosition, angle);
    std::vector<Point> goodPoints = filterPointsByVariances(slices_with_variance, rect_vertices, epsilon);

    if (isDebug) {
        Auxiliary::showCloudPoint(rect_vertices, points);
        Auxiliary::showCloudPoint(rect_vertices, pointsOutsideRect);
        Auxiliary::showCloudPoint(rect_vertices, goodPoints);
    }

    // calculate clustering of the remaining points
    int minSamples = 25;  // TODO - very important!!! original : 25
    auto rawNavigationPoints = std::vector<Point>{};
    while (rawNavigationPoints.empty() && minSamples > 0){
        rawNavigationPoints = getNavigationPoints(goodPoints, minSamples);
        minSamples-=5;
    }

    if (isDebug) {
        Auxiliary::showCloudPoint(rawNavigationPoints, points);
    }

    // filter and sort navigation points
    auto navigationPoints = filterCheckpoints(rawNavigationPoints, currentPosition);
    std::sort(navigationPoints.begin(), navigationPoints.end(), [&currentPosition](Point p1, Point p2) {
        return Auxiliary::calculateDistanceXY(p1, currentPosition) > Auxiliary::calculateDistanceXY(p2, currentPosition);
    });

    if (isDebug) {
        Auxiliary::showCloudPointAndExitPoints(navigationPoints, points);
    }

    std::cout << "amount of navigationPoints " << navigationPoints.size() << std::endl;
    return navigationPoints;
}


std::vector<Point> thoretic::filterPointsInsideRectangle(std::vector<Point> vertices, std::vector<Point> points) {
    auto pointsOutsideRectangle = std::vector<Point>{};
    int verticesAmount = vertices.size();
    for (auto point : points) {
        int amountOfCrossing = 0;
        for (int i = 0; i < verticesAmount; i++) {
            Point currentVertex = vertices[i];
            Point nextVertex = vertices[(i + 1) % verticesAmount];
            if ((currentVertex.x < point.x && point.x < nextVertex.x) ||
                (currentVertex.x > point.x && point.x > nextVertex.x)) {
                double ratio = (point.x - nextVertex.x) / (currentVertex.x - nextVertex.x);
                amountOfCrossing += ((ratio * currentVertex.y) + ((1 - ratio) * nextVertex.y)) >= point.y ? : 0;
            }
        }
        if (amountOfCrossing % 2 == 0) {
            pointsOutsideRectangle.push_back(point);
        }
    }
    return pointsOutsideRectangle;
}


std::vector<Point> thoretic::filterCheckpoints(std::vector<Point> navigationPoints, const Point &currentPosition, int minAngleDistance) {
    std::vector<std::pair<double, Point>> pointsWithAngles;
    auto center = currentPosition;
    std::sort(navigationPoints.begin(), navigationPoints.end(), [&center](const Point &p1, const Point &p2) {
        return Auxiliary::calculateDistanceXY(p1, center) > Auxiliary::calculateDistanceXY(p2, center);
    });
    for (Point point : navigationPoints) {
        double angle = Auxiliary::getAngleFromSlope((point.y - currentPosition.y) / (point.x - currentPosition.x));
        angle += angle < 0 ? 180 : 0;
        if (point.y < currentPosition.y) {
            while (angle < 180) {
                angle += 180;
            }
        }
        pointsWithAngles.push_back({angle, point});
    }
    std::vector<Point> goodCheckpoints;
    pointsWithAngles.push_back(pointsWithAngles.front());
    for (auto firstAngle = pointsWithAngles.begin(); firstAngle < pointsWithAngles.end() - 1; ++firstAngle) {
        bool toAdd = true;
        for (auto secondAngle = firstAngle + 1; secondAngle < pointsWithAngles.end(); secondAngle++) {
            if (abs(firstAngle->first - secondAngle->first) < minAngleDistance) {
                toAdd = false;
                break;
            }
        }
        if (toAdd) {
            goodCheckpoints.push_back(firstAngle->second);
        }
    }
    goodCheckpoints.push_back(pointsWithAngles.back().second);
    return goodCheckpoints;
}


std::vector<Point> thoretic::getNavigationPoints(std::vector<Point> goodPoints, int minSamples) {
    auto dbscan = DBSCAN(minSamples, 0.15, goodPoints);
    int numberOfClusters = dbscan.run();
    if (!numberOfClusters){
        return std::vector<Point>{};
    }
    std::vector<Point> clusteredPoints = dbscan.getPoints();
    std::vector<Point> navigationPoints;
    std::sort(clusteredPoints.begin(), clusteredPoints.end(), [](Point point1, Point point2) {
        return point1.label < point2.label;
    });
    int currentLabel = 1;
    auto it = std::find_if(clusteredPoints.begin(), clusteredPoints.end(), [](Point point) {
        return point.label == 1;
    });
    std::vector<Point> filteredClusteredPoints(it, clusteredPoints.end());
    std::vector<Point> cluster;
    for (auto point : filteredClusteredPoints) {
        if (point.label == currentLabel) {
            cluster.push_back(point);
        } else {
            navigationPoints.push_back(getNavigationPointFromCluster(cluster));
            currentLabel += 1;
            cluster.clear();
            cluster.push_back(point);
        }
    }
    //get from the last cluster
    navigationPoints.push_back(getNavigationPointFromCluster(cluster));
    return navigationPoints;
}


Point thoretic::getNavigationPointFromCluster(std::vector<Point> cluster){
    /*double maxDistanceToPolygon = -1;
    Point bestPoint;
    for (Point clusterPoint : cluster) {
        double distanceToPolygon = Auxiliary::getDistanceToClosestSegment(clusterPoint, edges);
        if (maxDistanceToPolygon < distanceToPolygon) {
            maxDistanceToPolygon = distanceToPolygon;
            bestPoint = clusterPoint;
        }
    }
    return bestPoint;*/
    return Auxiliary::GetCenterOfMass(cluster); // think about it !!!
}


std::vector<std::pair<Point, double>> thoretic::createPointsWithDistanceFromCurrentPosition(std::vector<Point> points, const Point &currentPosition) {
    auto pointsWithDistance = std::vector<std::pair<Point, double>>{};
    for (Point point : points) {
        pointsWithDistance.push_back({point, Auxiliary::calculateDistanceXY(currentPosition, point)});
    }
    return pointsWithDistance;
}


std::vector<std::pair<double, std::vector<Point>>> thoretic::getSlicesWithVariances(const std::vector<std::pair<Point, double>> pointsOutsidePolygonWithDistance, const Point &currentPosition, int angle) {
    auto pizzaSlices = Pizza::createPizzaSlices(currentPosition, pointsOutsidePolygonWithDistance, angle);
    std::vector<std::pair<double, std::vector<Point>>> slicesWithVariance;
    for (auto pizzaSlice : pizzaSlices) {
        int pizzaSliceSize = pizzaSlice.second.size();
        std::vector<Point> pizzaPoints;
        if (pizzaSliceSize > 2) {
            double sum = 0.0;
            for (auto point : pizzaSlice.second) {
                sum += point.second;
                pizzaPoints.push_back(point.first);
            }
            double mean = sum / pizzaSliceSize;
            double variance = 0.0;
            for (auto point : pizzaSlice.second) {
                variance += pow((point.second - mean), 2);
            }
            slicesWithVariance.push_back({variance / pizzaSliceSize, pizzaPoints});
        }
    }
    return slicesWithVariance;
}


std::vector<Point> thoretic::filterPointsByVariances(std::vector<std::pair<double, std::vector<Point>>> slices, std::vector<Point> rect, double epsilon) {
    std::vector<Point> goodPoints;
    std::vector<double> variances;
    for (auto slice : slices) {
        variances.push_back(slice.first);
    }
    auto minVariance = std::min_element(variances.begin(), variances.end());
    auto maxVariance = std::max_element(variances.begin(), variances.end());
    auto varianceDifference = *maxVariance - *minVariance;
    for (auto slice: slices) {
        double ratio = (slice.first - *minVariance) / varianceDifference;
        for (Point point : slice.second) {
            double minDistance = 10000;
            for (int i=0; i<rect.size(); i++) {
                Line edge(rect[i], rect[(i+1)%rect.size()]);
                double distance = Auxiliary::distanceBetweenPointAndSegment(point, edge);
                minDistance = distance < minDistance ? distance : minDistance;
            }
            if (minDistance > (1 - ratio) * epsilon) {
                goodPoints.push_back(point);
            }
        }
    }
    return goodPoints;
}
