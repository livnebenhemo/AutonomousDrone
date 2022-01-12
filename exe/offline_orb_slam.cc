
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

int main() {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string videoPath = data["offlineVideoTestPath"];
    ORB_SLAM2::System SLAM(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true);
    int amountOfAttepmpts = 2;

    cv::Ptr<cv::BackgroundSubtractor> pBackSub =
            cv::createBackgroundSubtractorMOG2(30, 100);
    while (amountOfAttepmpts--) {
        cv::VideoCapture capture(videoPath);
        if (!capture.isOpened()) {
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        //for (int i = 0; i < 170; ++i) {
        capture >> frame;
        //}
        cv::resize(frame, frame, cv::Size(960, 720));
        int amount_of_frames = 1;

        for (;;) {
            SLAM.TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC));
            capture >> frame;
            if (frame.empty()) {
                break;
            }
            std::cout << "frame:" << amount_of_frames++ << std::endl;
            cv::resize(frame, frame, cv::Size(960, 720));
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << std::endl;
        std::cout << amount_of_frames << std::endl;
        capture.release();
    }

    allMapPoints = SLAM.GetMap()->GetAllMapPoints();
    if (!allMapPoints.empty()) {
        saveMap(0);
    }
    sleep(20);
    SLAM.Shutdown();

    cvDestroyAllWindows();

    return 0;
}

