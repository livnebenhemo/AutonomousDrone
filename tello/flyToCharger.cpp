// #define _GLIBCXX_USE_CXX11_ABI 0
#include "Charger.h"
#include <opencv2/videoio.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

const char *const TELLO_STREAM_URL{"udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=5000"};

int main() {
    std::string chargerBluetoothAddress = "3C:61:05:03:81:E2";
    std::vector<std::pair<int, double>> markers;
    markers.push_back(std::pair<int, double>(4, 0.1815));
    markers.push_back(std::pair<int, double>(5, 0.1815));
    std::string telloYamlFilePath = "/home/pi/drone_charger/piCamera.yaml";
    //std::shared_ptr<ctello::Tello> drone = std::make_shared<ctello::Tello>();
    wiringPiSetup();
    pinMode(26,OUTPUT);
    digitalWrite(26,1);
    std::shared_ptr<cv::VideoCapture> capture = std::make_shared<cv::VideoCapture>();
    capture->set(32,3);
    capture->open(0);
    std::string takeOffCommand = "takeoff";
    //Charger charger(markers,chargerBluetoothAddress,capture,telloYamlFilePath,drone);
    int dronePort = 9000;
    while(dronePort > 8000){
    std::shared_ptr<ctello::Tello> drone = std::make_shared<ctello::Tello>();
    Charger charger(markers,chargerBluetoothAddress,capture,telloYamlFilePath,drone);
    drone->Bind(dronePort);
    drone->SendCommandWithResponse(takeOffCommand);
    charger.run();
    }
}
