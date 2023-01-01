//
// Created by tzuk on 02/01/2022.
//

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "include/Navigation.h"
#include "include/RRT.h"

std::vector<Point> Navigation::filterPointsByStartPosition(std::vector<Point> &points, std::pair<Point, Point> &track) {
    std::vector<Point> pointsInTrack;

    Point current = track.first;
    Point next = track.second;
    double trackLength = Auxiliary::calculateDistanceXY(current, next);
    for (const auto &point: points) {
        if (current.z < point.z || Auxiliary::calculateDistanceXY(current, point) > trackLength) {
            continue;
        }
        pointsInTrack.emplace_back(point);
    }
    return pointsInTrack;
}

bool
Navigation::objectDetection(std::vector<Point> &points, std::pair<Point, Point> &track, int sizeOfJump, bool debug) {
    std::string pangolinWindowName = "objectDetection" + std::to_string(std::rand());
    if (debug) {
        //Auxiliary::SetupPangolin(pangolinWindowName);
    }
    std::vector<Point> pointsInTrack;
    Point current = track.first;
    Point next = track.second;
    double trackLength = Auxiliary::calculateDistanceXY(current, next);
    for (const auto &point: points) {
        if (current.z < point.z || Auxiliary::calculateDistanceXY(current, point) > trackLength) {
            continue;
        }
        pointsInTrack.emplace_back(point);
    }
    std::vector<Point> pointsInFieldOfView;
    if (points.size() < sizeOfJump) {
        return true;
    }
    double trackAngle = atan2(next.y - current.y, next.x - current.x);
    for (const auto &point: pointsInTrack) {
        double pointAngle = atan2(point.y - current.y, point.x - current.x);
        double angle = Auxiliary::radiansToAngle(pointAngle - trackAngle);
        if (angle < 10 && angle > -10) {
            pointsInFieldOfView.emplace_back(point);
        }
    }
    if (pointsInFieldOfView.size() < sizeOfJump) {
        return true;
    }
    //Auxiliary::DrawMapPointsPangolin(points, pointsInFieldOfView, pangolinWindowName, track[1]);
    std::sort(pointsInFieldOfView.begin(), pointsInFieldOfView.end(),
              [&](const Point &point1, const Point &point2) -> bool {
                  return Auxiliary::calculateDistanceXY(point1, track.first) >
                         Auxiliary::calculateDistanceXY(point2, track.first);
              });//TODO: is it y axis only or distance from 0,0,0
    std::vector<double> z = Auxiliary::getZValues(pointsInFieldOfView);
    std::vector<double> y = Auxiliary::getYValues(pointsInFieldOfView);
    std::vector<double> x = Auxiliary::getXValues(pointsInFieldOfView);
    std::vector<double> pointsSizes;
    std::vector<double> variances;
    std::vector<std::pair<double, std::vector<Point>>> weightedPoints;

    while (y.size() > sizeOfJump) {
        double zVariance = Auxiliary::calculateVariance(z);
        double xVariance = Auxiliary::calculateVariance(x);
        double yVariance = Auxiliary::calculateVariance(y);
        variances.emplace_back(((zVariance) / (xVariance + yVariance)) * (double) y.size());
        pointsSizes.emplace_back(z.size());
        weightedPoints.emplace_back(((zVariance) / (xVariance + yVariance)) * (double) y.size(),
                                    std::vector<Point>(pointsInFieldOfView.begin(),
                                                       pointsInFieldOfView.begin() + (long) y.size()));
        z.resize(z.size() - sizeOfJump);
        x.resize(x.size() - sizeOfJump);
        y.resize(y.size() - sizeOfJump);
    }
    std::sort(weightedPoints.begin(), weightedPoints.end(),
              [](const std::pair<double, std::vector<Point>> &weightedPoint1,
                 const std::pair<double, std::vector<Point>> &weightedPoint2) -> bool {
                  return weightedPoint1.first > weightedPoint2.first;
              });
    if (debug) {
        //Auxiliary::DrawMapPointsPangolin(points, weightedPoints.front().second, pangolinWindowName, track);
        //Auxiliary::showGraph(pointsSizes, variances, "ro");
    }
    return weightedPoints.empty() || weightedPoints.front().second.size() < 10;
}

std::vector<Point> Navigation::getFloor(std::vector<Point> &points, unsigned long sizeOfJump) {
    std::sort(points.begin(), points.end(), [](const Point &point1, const Point &point2) -> bool {
        return point1.z > point2.z;
    });

    //Auxiliary::exportToXYZFile(points,
    //                         "/home/tzuk/Documents/AutonomousDroneResults/varianceFilter/cloud.xyz");
    std::vector<double> z = Auxiliary::getZValues(points);
    std::vector<double> y = Auxiliary::getYValues(points);
    std::vector<double> x = Auxiliary::getXValues(points);
    std::vector<double> pointsSizes;
    std::vector<double> variances;
    std::vector<std::pair<double, std::vector<Point>>> weightedPoints;
    while (z.size() > sizeOfJump) {
        double zVariance = Auxiliary::calculateVariance(z);
        double xVariance = Auxiliary::calculateVariance(x);
        double yVariance = Auxiliary::calculateVariance(y);
        variances.emplace_back(((xVariance + yVariance) / zVariance) * (double) z.size());
        pointsSizes.emplace_back(z.size());
        weightedPoints.emplace_back(((xVariance + yVariance) / zVariance) * (double) z.size(),
                                    std::vector<Point>(points.begin(), points.begin() + (long) z.size()));
        z.resize(z.size() - sizeOfJump);
        x.resize(x.size() - sizeOfJump);
        y.resize(y.size() - sizeOfJump);
    }
    std::sort(weightedPoints.begin(), weightedPoints.end(),
              [](const std::pair<double, std::vector<Point>> &weightedPoint1,
                 const std::pair<double, std::vector<Point>> &weightedPoint2) -> bool {
                  return weightedPoint1.first > weightedPoint2.first;
              });
    /*Auxiliary::exportToXYZFile(weightedPoints.front().second,
                               "/home/tzuk/Documents/AutonomousDroneResults/varianceFilter/floor.xyz");
    Auxiliary::showGraph(pointsSizes, variances, "ro");*/
    Auxiliary::showGraph(pointsSizes, variances, "ro");
    Auxiliary::SetupPangolin("floor");
    Auxiliary::DrawMapPointsPangolin(points, weightedPoints.front().second, "floor");
    return weightedPoints.front().second;
}

std::vector<Point> Navigation::dijkstra(Graph graph) {
    size_t startId = graph.verticesPositions.at(graph.start);
    size_t endId = graph.verticesPositions.at(graph.end);
    std::vector<size_t> positions;
    positions.reserve(graph.neighbors.size());
    for (const auto &neighbor: graph.neighbors) {
        positions.push_back(neighbor.first);
    }
    std::unordered_map<size_t, double> distances;
    std::unordered_map<size_t, size_t> previous;
    for (const auto position: positions) {
        if (!distances.count(position)) {
            distances.insert({position, std::numeric_limits<double>::max()});
        }
        if (!previous.count(position)) {
            previous.insert({position, std::numeric_limits<size_t>::max()});
        }
    }
    distances.at(startId) = 0;
    while (!positions.empty()) {
        auto currentNode = std::min_element(positions.begin(), positions.end(),
                                            [&distances](size_t &pos1, size_t &pos2) -> bool {
                                                return distances.at(pos1) < distances.at(pos2);
                                            });
        if (distances.at(*currentNode) == std::numeric_limits<double>::max()) {
            break;
        }
        for (const auto &neighbor: graph.neighbors.at(*currentNode)) {
            double newCost = distances.at(*currentNode) + neighbor.second;
            if (newCost < distances.at(neighbor.first)) {
                distances.at(neighbor.first) = newCost;
                previous.at(neighbor.first) = *currentNode;
            }
        }
        positions.erase(currentNode);
    }
    auto currentNode = endId;
    std::deque<Point> path;
    /*for (const auto prev: previous) {
        std::cout << "first: " << prev.first << std::endl;
        std::cout << "second: " << prev.second << std::endl;
    }*/
    while (previous.count(currentNode)) {
        path.push_front(graph.vertices[currentNode]);
        currentNode = previous.at(currentNode);
        //std::cout << "currentNode: " << currentNode << std::endl;
    }
    path.push_front(graph.start);
    return {path.begin(), path.end()};
}

std::vector<Point> Navigation::findOptPath(const std::vector<Point>& points, const Point &currentPoint){
    std::vector<int> indices(points.size());
    std::iota(indices.begin(), indices.end(), 0);//TODO: is there a better way to represent this?
    float min_sum = std::numeric_limits<double>::infinity();;
    std::vector<int> min_path_indices;
    do
    {
        float sum = Auxiliary::calculateDistanceXY(currentPoint, points[indices[0]]);
        for (int i = 0; i < indices.size(); i++){
            sum += Auxiliary::calculateDistanceXY(points[indices[i]], points[indices[(i+1)%indices.size()]]);
        }
        if (min_sum > sum) {
            min_sum = sum;
            min_path_indices = indices;
        }
    }
    while (std::next_permutation(indices.begin(), indices.end()));

    std::vector<Point> min_path;
    for (auto index: min_path_indices ){
        min_path.push_back(points[index]);
    }
    return min_path;
}


std::vector<Point> Navigation::createStarNavigation(const std::vector<Point>& navigationPoints, const Point &currentPoint) {
    std::vector<Point> starNavigationPoints;
    for (int i = 0; i < navigationPoints.size() - 1; ++i) {
        starNavigationPoints.emplace_back(currentPoint);
        starNavigationPoints.emplace_back(navigationPoints[i]);
    }
    starNavigationPoints.emplace_back(currentPoint);
    return starNavigationPoints;
}


std::vector<Point>
Navigation::getNavigationPathByRRT(std::vector<Point> &points, std::pair<Point, Point> &track, bool debug) {
    return std::vector<Point>{track.second};
    /*double pathLength = Auxiliary::calculateDistanceXY(track.first, track.second);
    std::cout << "path length: " << pathLength << std::endl;
    // TODO : previous numberOfIterations was 20000, check if work
    RRT rrt(track, points, debug, 10, 0.3, pathLength / 8);
    auto graph = rrt.BuildTrack();
    if (graph.start == Point() && graph.end == Point(1, 1, 1)) {
        std::cout << "cant find path" << std::endl;
        return {};
    }
    if (graph.vertices.size() <= 1) {
        std::cout << "straight line" << std::endl;
        if (debug) {
            std::cout << "livne" << std::endl;
            Auxiliary::SetupPangolin("path");
            Auxiliary::drawPathPangolin(points, graph.vertices, "path", track);
        }
        return std::vector<Point>{track.second};
    }
    auto path = dijkstra(graph);
    std::cout << "size of path: " << path.size() << std::endl;
    if (debug) {
        std::cout << "livne" << std::endl;
        Auxiliary::SetupPangolin("path");
        Auxiliary::drawPathPangolin(points, path, "path", track);
    }
    return path;*/
}
