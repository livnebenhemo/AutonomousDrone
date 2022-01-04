//
// Created by tzuk on 02/01/2022.
//

#include "tello/include/Navigation.h"


std::vector<Point>
Navigation::getValidNavigationPoints(const std::vector<Frame> &frames, const std::vector<Point> &points,
                                     const Point &currentLocation) {
    std::vector<Point> goodPoints;
    /*for (const auto &frame: frames) {
        for (const auto &point: frame.points) {
            if (point.z > frame.z) {
                goodPoints.emplace_back(point);
            }

        }
    }*/
    std::cout << "size for dbscan: " << points.size() * 0.001 << std::endl;
    DBSCAN dbscan(10, 0.0125, points, Auxiliary::calculateDistanceXZ);
    dbscan.run();
    std::unordered_map<int, std::vector<Point>> clusters;
    for (const auto &point: dbscan.getPoints()) {
        if (point.label == -1) {
            continue;
        }
        if (!clusters.count(point.label)) {
            clusters.insert({point.label, std::vector<Point>{point}});
        } else {
            clusters.at(point.label).emplace_back(point);
        }
    }
    std::cout << clusters.size() << std::endl;
    std::cout << points.size() << std::endl;
    std::pair<unsigned long, std::vector<Point>> biggestCluster;
    for (const auto &cluster: clusters) {
        Auxiliary::showCloudPoint(cluster.second, points);

        if (biggestCluster.first < cluster.second.size()) {
            biggestCluster.first = cluster.second.size();
            biggestCluster.second = cluster.second;
        }
    }
    std::cout << biggestCluster.second.size() << std::endl;
    Auxiliary::showCloudPoint(biggestCluster.second, points);
    Auxiliary::exportToXYZFile(biggestCluster.second);
    Auxiliary::exportToXYZFile(points, "/tmp/cloud.xyz");
}