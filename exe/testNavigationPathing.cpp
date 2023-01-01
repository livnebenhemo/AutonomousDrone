//
// Created by rbdstudent on 27/06/2021.
//

#include "include/Navigation.h"
#include<fstream>
#include <chrono>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include "../utils/include/Frame.h"
#include "include/RRT.h"
#include "include/Polygon.h"


std::pair<cv::Mat, cv::Mat> calculate_align_matrices(std::vector<cv::Point3d> points) {
    cv::Mat mu_align1;
    cv::Mat R_align;
    cv::reduce(points, mu_align1, 01, CV_REDUCE_AVG);

    cv::Point3d mu_align_pnt(mu_align1.at<double>(0), mu_align1.at<double>(1), mu_align1.at<double>(2));
    cv::Mat mu_align(mu_align_pnt);

    std::cout << "Centering points" << std::endl;
    for (auto &p: points) {
        p = p - mu_align_pnt;
    }

    cv::Mat A = Auxiliary::points3d_to_mat(points);
    cv::Mat w, u, vt;
    // cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    cv::SVDecomp(A, w, u, vt);
    R_align = u.t();


    return {R_align, mu_align};
}

std::vector<cv::Point3d> convertToCvPoints(const std::vector<Point> &points) {
    std::vector<cv::Point3d> cvPoints;
    for (const auto &point: points) {
        cvPoints.emplace_back(cv::Point3d(point.x, point.y, point.z));
    }
    return cvPoints;
}

std::vector<Point> convertCVToPoints(const std::vector<cv::Point3d> &cvpoints) {
    std::vector<Point> points;
    for (const auto &point: cvpoints) {
        points.emplace_back(Point(point.x, point.y, point.z));
    }
    return points;
}

cv::Mat convertPointToCVMat(const Point &point) {
    cv::Mat pnt3d(1, 3, CV_64FC1);
    pnt3d.at<double>(0, 0) = point.x;
    pnt3d.at<double>(0, 1) = point.y;
    pnt3d.at<double>(0, 2) = point.z;
    return pnt3d.t();
}

std::pair<cv::Mat, cv::Mat> align_map(std::vector<Point> &points) {
    auto[R_align, mu_align] = calculate_align_matrices(convertToCvPoints(points));

    for (auto &point: points) {
        auto pnt3d = convertPointToCVMat(point);
        cv::Mat align_pos;
        align_pos = R_align * (pnt3d - mu_align);

        point.x = align_pos.at<double>(0, 0);
        point.y = align_pos.at<double>(1, 0);
        point.z = align_pos.at<double>(2, 0);
    }
    return {R_align, mu_align};
}

std::vector<Point> getFramesFromFile(const std::string &fileName) {
    std::unordered_map<int, std::vector<Point>> framesMap;
    std::ifstream myFile(fileName);
    std::string line;
    std::vector<Point> allPoints;
    while (std::getline(myFile, line)) {
        std::stringstream lineStream(line);
        Point point;
        lineStream >> point.x;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.z;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.y;
        if (lineStream.peek() == ',') lineStream.ignore();

        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.frameId;

        allPoints.emplace_back(point);
    }
    return allPoints;

}

std::vector<Point> getPointsFromFile(const std::string &fileName) {
    std::ifstream myFile(fileName);
    std::string line;
    std::vector<Point> allPoints;
    while (std::getline(myFile, line)) {
        std::stringstream lineStream(line);
        Point point;
        lineStream >> point.x;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.z;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.y;
        /*if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qx;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qy;

        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qz;

        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qw;

        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.frameId;*/

        allPoints.emplace_back(point);
    }
    myFile.close();
    return allPoints;

}

std::vector<Point> getPointsFromXYZFile(const std::string &fileName) {
    std::ifstream myFile(fileName);
    std::string line;
    std::vector<Point> allPoints;
    while (std::getline(myFile, line)) {
        std::stringstream lineStream(line);
        Point point;
        lineStream >> point.x;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.z;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.y;
        allPoints.emplace_back(point);
    }
    return allPoints;

}

int main() {
    std::string datasetFilePath = Auxiliary::GetDataSetsDirPath() + "pointDataExtended.csv";
    auto points = getPointsFromFile(datasetFilePath);
    Polygon polygon(points, Point());
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Point> pathPoints = polygon.getExitPointsByPolygon(false);
    std::vector<Point> navigationPoints;
    navigationPoints.emplace_back(pathPoints[0]);
    Navigation navigation;
    for (int i = 0; i < pathPoints.size() - 1; ++i) {
        std::pair<Point, Point> track{pathPoints[i], pathPoints[i + 1]};
        auto result = navigation.getNavigationPathByRRT(points, track, false);
        navigationPoints.insert(navigationPoints.end(), result.begin(), result.end());
    }
    //auto filteredPoints = navigation.filterPointsByStartPosition(points.second,track);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    std::cout << "exe time in seconds: " << duration.count() * 0.000000001 << std::endl;
    std::cout << "find optimal path" << std::endl;
    navigationPoints = navigation.findOptPath(navigationPoints, Point());
    Auxiliary::showCloudPoint(navigationPoints, points);
    Auxiliary::SetupPangolin("full path");
    Auxiliary::drawPathPangolin(points, navigationPoints, "full path",
                                std::pair<Point, Point>{pathPoints[0], pathPoints[1]});
}