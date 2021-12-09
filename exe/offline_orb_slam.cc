//
// Created by ge on 14/11/19.
//
/**
 *
 * This file is a modification of ORB-SLAM2.
 *
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "tello/AutonomousDrone.h"

/************* SIGNAL *************/
#define ROWS 720
#define COLS 960
#define COLORS 3
std::vector<ORB_SLAM2::MapPoint *> allMapPoints;

void saveMap(ORB_SLAM2::System SLAM) {
    std::vector<ORB_SLAM2::MapPoint *> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
    for (auto p : mapPoints) {
        if (p != NULL) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z() << std::endl;
        }
    }
    pointData.close();
}

void saveMap(int fileNumber) {
    std::ofstream pointData;
    pointData.open("/tmp/pointData" + std::to_string(fileNumber) + ".csv");
    for (auto p : allMapPoints) {
        if (p != NULL) {
            auto frame = p->GetReferenceKeyFrame();
            int frameId = frame->mnFrameId;
            cv::Mat Tcw = frame->GetPose();
            auto point = p->GetWorldPos();
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
            auto q = ORB_SLAM2::Converter::toQuaternion(Rwc);
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z() << "," << q[0]
                      << "," << q[1] << "," << q[2] << "," << q[3] << "," << frameId <<
                      "," << twc.at<float>(0) << "," << twc.at<float>(1) << "," << twc.at<float>(2) << std::endl;
        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;

}

int main(int argc, char **argv) {
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
    int amountOfAttepmpts = 2;
    while (amountOfAttepmpts--) {
        cv::VideoCapture capture("/tmp/outpy.avi");
        if (!capture.isOpened()) {
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        capture >> frame;
        cv::resize(frame, frame, cv::Size(960, 720));
        int amount_of_frames = 1;
        for (;;) {
            SLAM.TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC));

            capture >> frame;
            if (frame.empty()) {
                break;
            }
            amount_of_frames++;
            cv::resize(frame, frame, cv::Size(960, 720));
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << std::endl;
        std::cout << amount_of_frames << std::endl;
        capture.release();
    }

    allMapPoints = SLAM.GetMap()->GetAllMapPoints();
    if (allMapPoints.size() > 0) {
        saveMap(0);
    }
    sleep(20);
    SLAM.Shutdown();

    cvDestroyAllWindows();

    return 0;
}

