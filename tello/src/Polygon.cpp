//
// Created by rbdstudent on 17/06/2021.
//

#include "tello/include/Polygon.h"

#include <utility>

Polygon::Polygon(std::vector<Point> points, const Point &polygonCenter, bool isExit) {
    this->points = std::move(points);
    this->isExit = isExit;
    this->polygonCenter = polygonCenter;
}

std::vector<Point> Polygon::getExitPointsByPolygon(bool isDebug) {
    //polygonCenter = Auxiliary::GetCenterOfMass(points);
    createPointsWithDistance();
    std::vector<std::pair<Point, double>> rawExitPoints = getRawPolygonCorners();
    double epsilon = 0.0;
    vertices = std::vector<Point>{};
    for (const auto &exitPoint: rawExitPoints) {
        epsilon += exitPoint.second;
        vertices.push_back(exitPoint.first);
    }
    epsilon /= rawExitPoints.size();
    if (isDebug) {
        Auxiliary::showCloudPoint(vertices, points);
    }
    smoothPolygon();
    filterPointsInsidePolygon();
    std::vector<Point> goodPoints = filterPointsByVariances(getSlicesWithVariances(angle), epsilon);
    if (isDebug) {
        Auxiliary::showCloudPoint(vertices, points);
        std::vector<Point> withOutVariances;
        for (const auto &point: pointsOutsidePolygon) {
            withOutVariances.emplace_back(point.first);
        }
        Auxiliary::showCloudPoint(vertices, withOutVariances);
        Auxiliary::showCloudPoint(vertices, goodPoints);
    }
    int minSamples = 25;
    auto rawNavigationPoints = std::vector<Point>{};
    while (rawNavigationPoints.empty() && minSamples > 0) {
        rawNavigationPoints = getNavigationPoints(goodPoints, minSamples);
        minSamples -= 5;
    }
    if (isDebug) {
        Auxiliary::showCloudPoint(rawNavigationPoints, points);
    }
    auto navigationPoints = filterCheckpoints(rawNavigationPoints);
    std::cout << "amount of navigationPoints " << navigationPoints.size() << std::endl;
    if (isDebug) {
        Auxiliary::showCloudPoint(navigationPoints, points);
    }
    if (isExit) {
        auto center = polygonCenter;
        std::sort(navigationPoints.begin(), navigationPoints.end(), [&center](const Point &p1, const Point &p2) {
            return Auxiliary::calculateDistanceXY(p1, center) > Auxiliary::calculateDistanceXY(p2, center);
        });
    }
    return navigationPoints;
}

std::vector<Point>
Polygon::filterCheckpoints(const std::vector<Point> &rawNavigationPoints, int minAngleDistance) const {
    std::vector<std::pair<double, Point>> pointsWithAngles;
    for (const Point &point: rawNavigationPoints) {
        double currentAngle = Auxiliary::getAngleFromSlope((point.y - polygonCenter.y) / (point.x - polygonCenter.x));
        currentAngle += currentAngle < 0 ? 180 : 0;
        if (point.y < polygonCenter.y) {
            while (currentAngle < 180) {
                currentAngle += 180;
            }
        }
        pointsWithAngles.emplace_back(currentAngle, point);
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

std::vector<Point> Polygon::getNavigationPoints(const std::vector<Point> &goodPoints, int minSamples) {
    auto dbscan = DBSCAN(minSamples, 0.15, goodPoints);
    int numberOfClusters = dbscan.run();
    if (!numberOfClusters) {
        return std::vector<Point>{};
    }
    std::vector<Point> clusteredPoints = dbscan.getPoints();
    std::vector<Point> navigationPoints;
    std::sort(clusteredPoints.begin(), clusteredPoints.end(), [](const Point &point1, const Point &point2) {
        return point1.label < point2.label;
    });
    int currentLabel = 1;
    auto it = std::find_if(clusteredPoints.begin(), clusteredPoints.end(), [](const Point &point) {
        return point.label == 1;
    });
    std::vector<Point> filteredClusteredPoints(it, clusteredPoints.end());
    std::vector<Point> cluster;
    for (const auto &point: filteredClusteredPoints) {
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

Point Polygon::getNavigationPointFromCluster(const std::vector<Point> &cluster) {
    double maxDistanceToPolygon = -1;
    Point bestPoint;
    for (const Point &clusterPoint: cluster) {
        double distanceToPolygon = Auxiliary::getDistanceToClosestSegment(clusterPoint, edges);
        if (maxDistanceToPolygon < distanceToPolygon) {
            maxDistanceToPolygon = distanceToPolygon;
            bestPoint = clusterPoint;
        }
    }
    return bestPoint;
}

std::vector<Point>
Polygon::filterPointsByVariances(const std::vector<std::pair<double, std::vector<Point>>> &slices, double epsilon) {
    std::vector<Point> goodPoints;
    std::vector<double> variances;
    for (const auto &slice: slices) {
        variances.push_back(slice.first);
    }
    auto minVariance = std::min_element(variances.begin(), variances.end());
    auto maxVariance = std::max_element(variances.begin(), variances.end());
    auto varianceDifference = *maxVariance - *minVariance;
    for (const auto &slice: slices) {
        double ratio = (slice.first - *minVariance) / varianceDifference;
        for (const Point &point: slice.second) {
            double minDistance = 10000;
            for (const auto &edge: edges) {
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

std::vector<std::pair<double, std::vector<Point>>> Polygon::getSlicesWithVariances(int currentAngle) {
    auto pizzaSlices = Pizza::createPizzaSlices(polygonCenter, pointsOutsidePolygon, currentAngle);
    std::vector<std::pair<double, std::vector<Point>>> slices;
    for (const auto &pizzaSlice: pizzaSlices) {
        int pizzaSliceSize = pizzaSlice.second.size();
        std::vector<Point> pizzaPoints;
        if (pizzaSliceSize > 2) {
            double sum = 0.0;
            for (const auto &point: pizzaSlice.second) {
                sum += point.second;
                pizzaPoints.push_back(point.first);
            }
            double mean = sum / pizzaSliceSize;
            double variance = 0.0;
            for (const auto &point: pizzaSlice.second) {
                variance += pow((point.second - mean), 2);
            }
            slices.emplace_back(variance / pizzaSliceSize, pizzaPoints);
        }
    }
    return slices;
}

void Polygon::createPointsWithDistance() {
    pointsWithDistance = std::vector<std::pair<Point, double>>{};
    for (const Point &point: points) {
        pointsWithDistance.emplace_back(point, Auxiliary::calculateDistanceXY(polygonCenter, point));
    }
}

void Polygon::filterPointsInsidePolygon() {
    pointsOutsidePolygon = std::vector<std::pair<Point, double>>{};
    int verticesAmount = vertices.size();
    for (const auto &point: pointsWithDistance) {
        int amountOfCrossing = 0;
        for (int i = 0; i < verticesAmount; i++) {
            Point currentVertex = vertices[i];
            Point nextVertex = vertices[(i + 1) % verticesAmount];
            if ((currentVertex.x < point.first.x && point.first.x < nextVertex.x) ||
                (currentVertex.x > point.first.x && point.first.x > nextVertex.x)) {
                double ratio = (point.first.x - nextVertex.x) / (currentVertex.x - nextVertex.x);
                amountOfCrossing += ((ratio * currentVertex.y) + ((1 - ratio) * nextVertex.y)) >= point.first.y ?
                                    1 : 0;
            }
        }
        if (amountOfCrossing % 2 == 0) {
            pointsOutsidePolygon.push_back(point);
        }
    }
}

void Polygon::smoothPolygon(int angleRange) {
    vertices.push_back(vertices[0]);
    bool stop = false;
    while (!stop) {
        edges = std::vector<Line>{};
        for (int i = 1; i < vertices.size(); ++i) {
            edges.emplace_back(Line(vertices[i - 1], vertices[i]));

        }
        for (int i = 1; i < edges.size(); ++i) {
            Line fromEdge = edges[i - 1];
            double currentAngle = Auxiliary::getAngleBySlopes(fromEdge, edges[i]);
            if (!(currentAngle > angleRange && currentAngle < 180 - angleRange)) {
                for (auto verticesIt = vertices.begin(); verticesIt < vertices.end(); verticesIt++) {
                    if (*verticesIt == fromEdge.getPoint2()) {
                        vertices.erase(verticesIt);
                        break;
                    }
                }
                break;
            }
            if (i == edges.size() - 1) {
                stop = true;
            }
        }
    }
}

std::vector<std::pair<Point, double>> Polygon::getRawPolygonCorners() {
    std::vector<Line> lines = Pizza::createPizzaLines(polygonCenter, angle);
    auto slices = Pizza::createPizzaSlices(polygonCenter, pointsWithDistance, angle);
    std::vector<std::pair<Point, double>> polygonVertices;
    auto sortRule = [](const std::pair<Point, double> &point1, const std::pair<Point, double> &point2) -> bool {
        return point2.second < point1.second;
    };
    for (auto slice: slices) {
        std::sort(slice.second.begin(), slice.second.end(), sortRule);
        std::pair<Point, double> medianPoint = slice.second[slice.second.size() * 0.5];
        polygonVertices.push_back(medianPoint);
    }
    return polygonVertices;
}