#include "AutonomousDrone.h"

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
    std::shared_ptr<ctello::Tello> drone = std::make_shared<ctello::Tello>();
    drone->Bind();

    while (!drone->SendCommandWithResponse("streamon"));
    std::thread cameraThread(getCameraFeed);
    while (!cameraOpen){
        usleep(5000000);
    }
    std::vector<std::pair<int, double>> markers;
    markers.emplace_back(std::pair<int, double>(4, 0.1815));
    markers.emplace_back(std::pair<int, double>(5, 0.1815));
    markers.emplace_back(std::pair<int, double>(6, 0.1815));

    Charger charger(markers,holdCamera,"TELLO-9F5EC2","/home/rbdstudent/ORB_SLAM2/ORB_SLAM2/Examples/Monocular/tello_aruco_9F5EC2.yaml",
                    drone,frame,9000);
    drone->SendCommandWithResponse("takeoff");
    sleep(2);
    charger.chargeByPaper();
    return 0;
}