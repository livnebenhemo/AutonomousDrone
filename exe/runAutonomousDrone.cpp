//
// Created by rbdstudent on 30/06/2021.
//
#include "tello/include/AutonomousDrone.h"
#include <nlohmann/json.hpp>

int main() {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    std::shared_ptr<ctello::Tello> drone = std::make_shared<ctello::Tello>(true);
    while (!drone->SendCommandWithResponseByThread("streamon"));
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string droneYamlPathAruco = data["DroneYamlPathAruco"];
    std::string droneName = data["DroneName"];
    bool withPlot = data["WithPlot"];
    int sizeOfFrameStack = data["sizeOfFrameStack"];

    AutonomousDrone autonomousDrone(drone, vocPath, droneYamlPathSlam, droneYamlPathAruco, droneName, sizeOfFrameStack,
                                    withPlot);
    autonomousDrone.run();

}
