//
// Created by rbdstudent on 17/06/2021.
//

#include "Polygon.h"

Polygon::Polygon(std::vector<Point> points, bool isExit) {
    this->points = points;
    this->isExit = isExit;
}

std::vector<Point> Polygon::getExitPointsByPolygon(bool isDebug) {
    polygonCenter = Auxiliary::GetCenterOfMass(points);
    createPointsWithDistance();
    std::vector<std::pair<Point, double>> rawExitPoints = getRawPolygonCorners();
    double epsilon = 0.0;
    vertices = std::vector<Point>{};
    for (auto exitPoint : rawExitPoints) {
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
        for (auto point : pointsOutsidePolygon) {
            withOutVariances.emplace_back(point.first);
        }
        Auxiliary::showCloudPoint(vertices, withOutVariances);
        Auxiliary::showCloudPoint(vertices, goodPoints);
    }
    auto rawNavigationPoints = getNavigationPoints(goodPoints);
    if (isDebug) {
        std::cout << "amount of raw navigationPoints" << rawNavigationPoints.size() << std::endl;
        Auxiliary::showCloudPoint(rawNavigationPoints, points);
    }
    auto navigationPoints = filterCheckpoints(rawNavigationPoints);

    if (isDebug) {
        std::cout << "amount of navigationPoints" << navigationPoints.size() << std::endl;
        Auxiliary::showCloudPoint(navigationPoints, points);
    }
    if (isExit) {
        auto center = polygonCenter;
        std::sort(navigationPoints.begin(), navigationPoints.end(), [&center](Point p1, Point p2) {
            return Auxiliary::calculateDistance(p1, center) > Auxiliary::calculateDistance(p2, center);
        });
    }
    return navigationPoints;
}

std::vector<Point> Polygon::filterCheckpoints(std::vector<Point> rawNavigationPoints, int minAngleDistance) {
    auto polygonCenterCopy = polygonCenter;
    /*std::sort(rawNavigationPoints.begin(),rawNavigationPoints.end(),[polygonCenterCopy](Point point1,Point point2){
        return Auxiliary::calculateDistance(polygonCenterCopy,point1) > Auxiliary::calculateDistance(polygonCenterCopy,point2);
    });*/
    std::vector<std::pair<double, Point>> pointsWithAngles;
    for (Point point : rawNavigationPoints) {
        double angle = Auxiliary::getAngleFromSlope((point.y - polygonCenter.y) / (point.x - polygonCenter.x));
        angle += angle < 0 ? 180 : 0;
        if (point.y < polygonCenter.y) {
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

std::vector<Point> Polygon::getNavigationPoints(std::vector<Point> goodPoints, int minSamples) {
    auto dbscan = DBSCAN(minSamples, 0.15, goodPoints);
    int numberOfClusters = dbscan.run();
    if (numberOfClusters == 1) {
        dbscan.setMinPoints(10);
        numberOfClusters = dbscan.run();
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
            double maxDistanceToPolygon = -1;
            for (Point clusterPoint : cluster) {
                double distanceToPolygon = Auxiliary::getDistanceToClosestSegment(clusterPoint, edges);
                if (maxDistanceToPolygon < distanceToPolygon) {
                    maxDistanceToPolygon = distanceToPolygon;
                    if (navigationPoints.size() < currentLabel) {
                        navigationPoints.insert(navigationPoints.begin() + currentLabel - 1, Point(clusterPoint));
                    } else {
                        navigationPoints[currentLabel - 1] = Point(clusterPoint);
                    }
                }
            }
            currentLabel += 1;
            cluster.clear();
            cluster.push_back(point);
        }
    }

    return navigationPoints;
}

std::vector<Point>
Polygon::filterPointsByVariances(std::vector<std::pair<double, std::vector<Point>>> slices, double epsilon) {
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
            for (auto edge : edges) {
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

std::vector<std::pair<double, std::vector<Point>>> Polygon::getSlicesWithVariances(int angle) {
    auto pizzaSlices = Pizza::createPizzaSlices(polygonCenter, pointsOutsidePolygon, angle);
    std::vector<std::pair<double, std::vector<Point>>> slices;
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
            slices.push_back({variance / pizzaSliceSize, pizzaPoints});
        }
    }
    return slices;
}

void Polygon::createPointsWithDistance() {
    pointsWithDistance = std::vector<std::pair<Point, double>>{};
    for (Point point : points) {
        pointsWithDistance.push_back({point, Auxiliary::calculateDistance(polygonCenter, point)});
    }
}

void Polygon::filterPointsInsidePolygon() {
    pointsOutsidePolygon = std::vector<std::pair<Point, double>>{};
    int verticesAmount = vertices.size();
    for (auto point : pointsWithDistance) {
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
    auto sortRule = [](std::pair<Point, double> point1, std::pair<Point, double> point2) -> bool {
        return point2.second < point1.second;
    };
    for (auto slice : slices) {
        std::sort(slice.second.begin(), slice.second.end(), sortRule);
        std::pair<Point, double> medianPoint = slice.second[slice.second.size() * 0.5];
        polygonVertices.push_back(medianPoint);
    }
    return polygonVertices;
}