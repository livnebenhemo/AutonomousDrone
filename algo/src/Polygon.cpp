//
// Created by rbdstudent on 17/06/2021.
//

#include "include/Polygon.h"
#include "include/Navigation.h"

#include <utility>

Polygon::Polygon(std::vector<Point> points, const Point &polygonCenter, bool isExit) {
    this->points = std::move(points);
    this->isExit = isExit;
    this->polygonCenter = polygonCenter;
}

std::vector<Point> Polygon::getExitPointsByPolygon(bool isDebug) {
    //polygonCenter = Auxiliary::GetCenterOfMass(points);
    createPointsWithDistance(points);
    std::vector<std::pair<Point, double>> rawExitPoints = getRawPolygonCorners();
    vertices = std::vector<Point>{};
    for (const auto &exitPoint: rawExitPoints) {
        vertices.push_back(exitPoint.first);
    }
    if (isDebug) {
        Auxiliary::showCloudPoint(vertices, points);
    }
    //smoothPolygon();
    // filterPointsInsidePolygon();
    std::vector<Point> goodPoints{};
    for (const auto &outSidePoint: pointsOutsidePolygon) {
        goodPoints.push_back(outSidePoint.first);
    }
    /*if (isDebug) {
        Auxiliary::showCloudPoint(vertices, points);
        std::vector<Point> withOutVariances;
        for (const auto &point: pointsOutsidePolygon) {
            withOutVariances.emplace_back(point.first);
        }
        //Auxiliary::showCloudPoint(vertices, withOutVariances);
        //Auxiliary::showCloudPoint(vertices, goodPoints);
    }*/
    auto rawNavigationPoints = getNavigationPointsByVertexSharpAngle(goodPoints, 90);

    /*int minSamples = 10;
    auto rawNavigationPoints = std::vector<Point>{};
    while (rawNavigationPoints.empty() && minSamples > 0) {
        rawNavigationPoints = getNavigationPointsByVertexSharpAngle(goodPoints, 90);
        minSamples -= 5;
    }*/
    if (isDebug) {
        std::cout << "nav" << std::endl;
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
        navigationPoints = std::vector<Point>{navigationPoints.front()};
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

std::vector<Point> Polygon::getNavigationPointsByVertexSharpAngle(const std::vector<Point> &goodPoints, int maxAngle) {
    std::unordered_map<Point, double> goodVertex;
    for (int i = 0; i < vertices.size() - 2; ++i) {
        Line line1(vertices[i], vertices[i + 1]);
        Line line2(vertices[i + 1], vertices[i + 2]);
        double angle = Auxiliary::getAngleBySlopes(line1, line2);
        if (0 < angle && angle < maxAngle) {
            std::cout << "angle: " << angle << std::endl;
            goodVertex.insert({vertices[i + 1], angle});
        }
    }
    auto verticesSize = vertices.size();
    Line line1(vertices[verticesSize - 3], vertices[verticesSize - 2]);
    Line line2(vertices[verticesSize - 2], vertices[verticesSize - 1]);
    double angle = Auxiliary::getAngleBySlopes(line1, line2);
    if (0 < angle && angle < maxAngle) {
        std::cout << "angle1: " << angle << std::endl;

        goodVertex.insert({vertices[verticesSize - 2], angle});
    }

    Line line3(vertices[verticesSize - 2], vertices[verticesSize - 1]);
    Line line4(vertices[verticesSize - 1], vertices[0]);
    angle = Auxiliary::getAngleBySlopes(line3, line4);
    if (0 < angle && angle < maxAngle) {
        std::cout << "angle2: " << angle << std::endl;

        goodVertex.insert({vertices[verticesSize - 1], angle});
    }

    Line line5(vertices[verticesSize - 1], vertices[0]);
    Line line6(vertices[0], vertices[1]);
    angle = Auxiliary::getAngleBySlopes(line5, line6);
    if (0 < angle && angle < maxAngle){
        std::cout << "angle3: " << angle << std::endl;

        goodVertex.insert({vertices[0], angle});
    }
    std::cout << goodVertex.size() << std::endl;
    std::vector<Point> goodVertexVector;
    for (auto[point, angle]: goodVertex) {
        goodVertexVector.emplace_back(point);
    }
    return goodVertexVector;
}

std::vector<Point> Polygon::getNavigationPoints(const std::vector<Point> &goodPoints, int minSamples) {
    auto dbscan = DBSCAN(minSamples, 0.1, goodPoints);
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
        auto pizzaSliceSize = (double) pizzaSlice.second.size();
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

void Polygon::createPointsWithDistance(const std::vector<Point> &CurrentPoints) {
    pointsWithDistance = std::vector<std::pair<Point, double>>{};
    for (const Point &point: CurrentPoints) {
        pointsWithDistance.emplace_back(point, Auxiliary::calculateDistanceXY(polygonCenter, point));
    }
}

void Polygon::filterPointsInsidePolygon() {
    pointsOutsidePolygon = std::vector<std::pair<Point, double>>{};
    size_t verticesAmount = vertices.size();
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
    //std::vector<Line> lines = Pizza::createPizzaLines(polygonCenter, angle);
    auto slices = Pizza::createPizzaSlices(polygonCenter, pointsWithDistance, pizzaAngle);
    std::vector<std::pair<int, std::vector<std::pair<Point, double>>>> vectorSlices;
    for (const auto &slice: slices) {
        vectorSlices.emplace_back(slice);
    }
    std::sort(vectorSlices.begin(), vectorSlices.end(),
              [](std::pair<int, std::vector<std::pair<Point, double>>> &slice1,
                 std::pair<int, std::vector<std::pair<Point, double>>> &slice2) -> bool {
                  return slice1.first < slice2.first;
              });
    std::vector<std::pair<Point, double>> polygonVertices;
    auto sortRule = [](const std::pair<Point, double> &point1, const std::pair<Point, double> &point2) -> bool {
        return point2.second > point1.second;
    };
    for (auto &slice: vectorSlices) {
        std::sort(slice.second.begin(), slice.second.end(), sortRule);
        Point point((polygonCenter.x + slice.second.back().first.x) / 2,
                    (polygonCenter.y + slice.second.back().first.y) / 2, polygonCenter.z);
        std::pair<Point, double> medianPoint = {point,
                                                slice.second.back().second / 2};//slice.second[slice.second.size() / 2];
        polygonVertices.emplace_back(medianPoint);

        /*std::vector<Point> slicePoints{};
        for (const auto &slicePoint: slice.second) {
            slicePoints.emplace_back(slicePoint.first);
        }
        Point meanPoint = Auxiliary::GetCenterOfMass(slicePoints);
        polygonVertices.emplace_back(meanPoint, Auxiliary::calculateDistanceXY(polygonCenter, meanPoint));*/
    }
    return polygonVertices;
}