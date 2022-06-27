
#include <nlohmann/json.hpp>
#include "../tello/include/AutonomousDrone.h"

/************* SIGNAL *************/
#define ROWS 720
#define COLS 960
#define COLORS 3

void saveMap(ORB_SLAM2::System &SLAM) {
    auto mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
    for (auto &p: mapPoints) {
        if (p != nullptr) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z() << std::endl;
        }
    }
    pointData.close();
}

void saveFrame(cv::Mat &img, cv::Mat &pose, int frameCount, ORB_SLAM2::System &SLAM, std::string &resultFolderPath) {
    std::ofstream frameData;
    int currentFrameId = SLAM.GetTracker()->mCurrentFrame.mnId;
    frameData.open(resultFolderPath + "frameData" +
                   std::to_string(currentFrameId) + ".csv");

    cv::Mat Rwc = pose.rowRange(0, 3).colRange(0, 3);
    cv::Mat twc = -Rwc.t() * pose.rowRange(0, 3).col(3);
    frameData << currentFrameId << ',' << twc.at<float>(0) << ',' << twc.at<float>(2) << ',' << twc.at<float>(1) << ','
              << Rwc.at<float>(0, 0) << ',' << Rwc.at<float>(0, 1) << ',' << Rwc.at<float>(0, 2)
              << ',' << Rwc.at<float>(1, 0) << ',' << Rwc.at<float>(1, 1) << ',' << Rwc.at<float>(1, 2) << ','
              << Rwc.at<float>(2, 0)
              << ',' << Rwc.at<float>(2, 1) << ',' << Rwc.at<float>(2, 2) << std::endl;
    cv::imwrite(
            resultFolderPath + "frame_" + std::to_string(currentFrameId) + "_" + std::to_string(frameCount) + ".png",
            img);
    frameData.close();
}

void saveMap(std::string &resultFolderPath, int mapNumber, ORB_SLAM2::System &SLAM) {
    std::ofstream pointData;
    pointData.open(resultFolderPath + "cloud" + std::to_string(mapNumber) + ".csv");
    int amountOfPoints;
    for (auto &p: SLAM.GetMap()->GetAllMapPoints()) {
        if (p != nullptr && !p->isBad()) {
            auto frameId = p->GetReferenceKeyFrame()->mnFrameId;
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z() << "," << frameId << std::endl;
            amountOfPoints++;
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
    char currentDirPath[256];
    getcwd(currentDirPath, 256);

    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));
    std::string currentTime(time_buf);
    std::filesystem::create_directory("../simulatorDataSets/" + currentTime);
    std::string currentWorkingDir = std::string(currentDirPath) + "/../simulatorDataSets/" + currentTime + "/";
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string videoPath = data["offlineVideoTestPath"];
    bool loadMap = data["loadMap"];
    bool isSavingMap = data["saveMap"];
    std::string loadMapPath = data["loadMapPath"];
    std::string saveMapPath = data["saveMapPath"];
    ORB_SLAM2::System SLAM(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, loadMap, loadMapPath, true);
    int amountOfAttepmpts = 0;
    while (amountOfAttepmpts++ < 1) {
        cv::VideoCapture capture(videoPath);
        if (!capture.isOpened()) {
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        for (int i = 0; i < 170; ++i) {
            capture >> frame;

        }
        int amount_of_frames = 1;

        for (;;) {
            auto pose = SLAM.TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC));
            if (!pose.empty()) {
                saveFrame(frame, pose, amount_of_frames++, SLAM, currentWorkingDir);
            }
            capture >> frame;

            if (frame.empty()) {
                break;
            }
        }
        saveMap(currentWorkingDir, amountOfAttepmpts, SLAM);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << std::endl;
        std::cout << amount_of_frames << std::endl;
        capture.release();
    }

    saveMap(currentWorkingDir, amountOfAttepmpts, SLAM);
    if(isSavingMap){
        SLAM.SaveMap(saveMapPath);
    }
    //sleep(20);
    SLAM.Shutdown();

    cvDestroyAllWindows();

    return 0;
}

