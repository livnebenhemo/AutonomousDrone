#include <nlohmann/json.hpp>
#include "tello/include/AutonomousDrone.h"

std::shared_ptr<bool> holdCamera;
std::shared_ptr<cv::Mat> frame;
bool cameraOpen = false;

void getCameraFeed() {
    frame = std::make_shared<cv::Mat>();
    holdCamera = std::make_shared<bool>(false);
    cv::VideoCapture capture("udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=5000", cv::CAP_FFMPEG);
    while (true) {
        try {
            if (!*holdCamera) {
                capture.read(*frame);
                cv::resize(*frame,*frame,cv::Size(360,240));
                if (!frame->empty()) {
                    cameraOpen = true;
                    imshow("CTello Stream", *frame);
                    if (cv::waitKey(1) == 27) {
                        break;
                    }
                }
            } else {
                sleep(1);
            }
        } catch (...) {
            break;
        }
    }
}


int main() {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    std::shared_ptr<ctello::Tello> drone = std::make_shared<ctello::Tello>();
    drone->Bind();
    std::string droneYamlPathAruco = data["DroneYamlPathAruco"];
    std::string droneName = data["DroneName"];
    while (!drone->SendCommandWithResponse("streamon"));
    std::thread cameraThread(getCameraFeed);
    while (!cameraOpen) {
        usleep(5000000);
    }
    std::vector<std::pair<int, double>> markers;
    markers.emplace_back(std::pair<int, double>(4, 0.1815));
    markers.emplace_back(std::pair<int, double>(5, 0.1815));
    markers.emplace_back(std::pair<int, double>(6, 0.1815));
    Charger charger(markers, holdCamera, droneName, droneYamlPathAruco,
                    drone, frame, 9000);
    drone->SendCommandWithResponse("takeoff");
    charger.navigateToBox();
    return 0;
}