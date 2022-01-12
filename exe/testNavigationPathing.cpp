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

cv::Mat points3d_to_mat(const std::vector<cv::Point3d> &points3d) {
    std::size_t nPoints = points3d.size();
    cv::Mat mat((int) nPoints, 3, CV_64FC1);
    for (std::size_t i = 0; i < nPoints; i++) {
        mat.at<double>(i, 0) = points3d[i].x;
        mat.at<double>(i, 1) = points3d[i].y;
        mat.at<double>(i, 2) = points3d[i].z;
    }

    return mat.t();
}

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

    cv::Mat A = points3d_to_mat(points);
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

std::pair<std::vector<Frame>, std::vector<Point>> getFramesFromFile(const std::string &fileName) {
    std::unordered_map<int, std::vector<Point>> framesMap;
    std::ifstream myFile(fileName);
    std::string line;
    std::vector<Frame> frames;
    std::vector<Point> allPoints;
    while (std::getline(myFile, line)) {
        std::stringstream lineStream(line);
        Point point;
        Frame frame;
        lineStream >> point.x;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.z;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.y;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qx;
        frame.qx = point.qx;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qy;
        frame.qy = point.qy;

        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qz;
        frame.qz = point.qz;

        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qw;
        frame.qw = point.qw;

        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.frameId;
        frame.frameId = point.frameId;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> frame.x;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> frame.z;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> frame.y;
        if (!framesMap.count(point.frameId)) {
            framesMap.insert({point.frameId, std::vector<Point>{point}});
        } else {
            framesMap.at(point.frameId).emplace_back(point);
        }
        allPoints.emplace_back(point);
        frames.emplace_back(frame);
    }
    std::unordered_map<int, Frame> returnedFrames;
    std::vector<Frame> frames2;
    for (const auto &frame: frames) {
        if (!returnedFrames.count(frame.frameId)) {
            auto points = framesMap.at(frame.frameId);
            returnedFrames.insert({frame.frameId,
                                   Frame(frame.x, frame.y, frame.z, frame.qx, frame.qy, frame.qz, frame.qw,
                                         frame.frameId, points,
                                         points.size())});
            frames2.emplace_back(
                    Frame(frame.x, frame.y, frame.z, frame.qx, frame.qy, frame.qz, frame.qw, frame.frameId, points,
                          points.size()));
        }
    }
    return {frames2, allPoints};

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
    std::string datasetFilePath = Auxiliary::GetDataSetsDirPath() + "buildings/pointData0.csv";
    auto points = getFramesFromFile(datasetFilePath);
    auto[R, T] = align_map(points.second);
    //Auxiliary::DrawMapPointsPangolin(points.second,{},Point(0.2,1,-0.1));
    auto start = std::chrono::high_resolution_clock::now();

    Navigation navigation;
    std::pair<Point, Point> track{Point(0, 0, -0.05), Point(0.3, 1, -0.1)};
    navigation.objectDetection(points.second, track, false);
    //navigation.getFloor(points.second, points.second.size() / 100);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << duration.count() << std::endl;
}