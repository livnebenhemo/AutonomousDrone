//
// Created by tzuk on 16/01/2022.
//

#include <nlohmann/json.hpp>
#include "../tello/include/AutonomousDrone.h"

/************* SIGNAL *************/
#define ROWS 720
#define COLS 960
#define COLORS 3
std::vector<ORB_SLAM2::MapPoint *> allMapPoints;

void saveMap(ORB_SLAM2::System SLAM) {
    std::vector<ORB_SLAM2::MapPoint *> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
    for (auto p: mapPoints) {
        if (p != nullptr) {
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
    for (auto p: allMapPoints) {
        if (p != nullptr) {
            auto frame = p->GetReferenceKeyFrame();
            auto frameId = frame->mnFrameId;
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

void stopProgramHandler(int s) {
    saveMap(std::chrono::steady_clock::now().time_since_epoch().count());
    cvDestroyAllWindows();
    std::cout << "stoped program" << std::endl;
    exit(1);
}

int main() {
    signal(SIGINT, stopProgramHandler);
    signal(SIGTERM, stopProgramHandler);
    signal(SIGABRT, stopProgramHandler);
    signal(SIGSEGV, stopProgramHandler);
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    int videoPath = data["onlineVideoPath"];
    system("v4l2-ctl -d /dev/video0 -c exposure_auto=0");
    system("v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0");
    ORB_SLAM2::System SLAM(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true);
    cv::VideoCapture capture(videoPath);
    if (!capture.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return 0;
    } else {
        std::cout << "Success opening video stream or file" << std::endl;
    }
    cv::Mat frame;
    capture >> frame;
    cv::Size size(640, 480);
    cv::resize(frame, frame, size);
    for (;;) {
        SLAM.TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC));
        allMapPoints = SLAM.GetMap()->GetAllMapPoints();
        capture >> frame;
        if (frame.empty()) {
            break;
        }
        cv::resize(frame, frame, size);
    }
    capture.release();

    if (!allMapPoints.empty()) {
        saveMap(0);
    }
    sleep(20);
    SLAM.Shutdown();

    cvDestroyAllWindows();

    return 0;
}

