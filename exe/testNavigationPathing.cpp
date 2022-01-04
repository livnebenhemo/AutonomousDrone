//
// Created by rbdstudent on 27/06/2021.
//

#include "include/Navigation.h"
#include<fstream>
#include <chrono>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include "tello/include/Frame.h"

cv::Mat points3d_to_mat(const std::vector<cv::Point3f> &points3d) {
    std::size_t nPoints = points3d.size();
    cv::Mat mat((int) nPoints, 3, CV_32F);
    for (std::size_t i = 0; i < nPoints; i++) {
        mat.at<float>(i, 0) = points3d[i].x;
        mat.at<float>(i, 1) = points3d[i].y;
        mat.at<float>(i, 2) = points3d[i].z;
    }

    return mat.t();
}

std::pair<cv::Mat, cv::Mat> calculate_align_matrices(std::vector<cv::Point3f> points) {
    cv::Mat mu_align1;
    cv::Mat R_align;
    cv::reduce(points, mu_align1, 01, CV_REDUCE_AVG);

    cv::Point3f mu_align_pnt(mu_align1.at<float>(0), mu_align1.at<float>(1), mu_align1.at<float>(2));
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
    std::cout << "R_align:" << R_align << std::endl;


    return {R_align, mu_align};
}

std::vector<cv::Point3f> convertToCvPoints(const std::vector<Point> &points) {
    std::vector<cv::Point3f> cvPoints;
    for (const auto &point: points) {
        cvPoints.emplace_back(cv::Point3f(point.x, point.y, point.z));
    }
    return cvPoints;
}

std::vector<Point> convertCVToPoints(const std::vector<cv::Point3f> &cvpoints) {
    std::vector<Point> points;
    for (const auto &point: cvpoints) {
        points.emplace_back(Point(point.x, point.y, point.z));
    }
    return points;
}

cv::Mat convertPointToCVMat(const Point &point) {
    cv::Mat pnt3d(4, 4, CV_64FC1);
    Eigen::Quaterniond q(point.qw, point.qx, point.qy, point.qz);
    auto rot = q.toRotationMatrix();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pnt3d.at<double>(i, j) = rot(i, j);
        }
    }
    pnt3d.at<double>(0, 3) = point.x;
    pnt3d.at<double>(1, 3) = point.y;
    pnt3d.at<double>(2, 3) = point.z;
    pnt3d.at<double>(3, 0) = 0;
    pnt3d.at<double>(3, 1) = 0;
    pnt3d.at<double>(3, 2) = 0;
    pnt3d.at<double>(3, 3) = 1;
    return pnt3d;
}

std::pair<cv::Mat, cv::Mat> align_map(std::vector<Point> &points) {
    auto[R_align, mu_align] = calculate_align_matrices(convertToCvPoints(points));

    for (auto &point: points) {
        auto pnt3d = convertPointToCVMat(point);
        std::cout << "pnt3d: " << pnt3d << std::endl;
        cv::Mat align_pos;
        // Align xyz:
        align_pos = R_align * (pnt3d - mu_align);
        cv::Mat cvMat3 = align_pos.rowRange(0, 3).colRange(0, 3);

        Eigen::Matrix<double, 3, 3> eigMat;

        eigMat << cvMat3.at<double>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
                cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
                cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);
        Eigen::Quaterniond q(eigMat);
        point.x = align_pos.at<double>(0, 3);
        point.y = align_pos.at<double>(1, 3);
        point.z = align_pos.at<double>(2, 3);
        point.qw = q.w();
        point.qx = q.x();
        point.qy = q.y();
        point.qz = q.z();
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

int main() {
    auto points = getFramesFromFile("/tmp/pointData2.csv");
    auto start = std::chrono::high_resolution_clock::now();
    auto[R, T] = align_map(points.second);
    Navigation navigation;
    navigation.getValidNavigationPoints(points.first, points.second, Point(0.1, 0.1, -0.15));
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;
}