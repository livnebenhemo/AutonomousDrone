//
// Created by rbdstudent on 30/06/2021.
//
#include "include/AutonomousDrone.h"
#include <nlohmann/json.hpp>

int main() {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    std::string droneName = data["DroneName"];
    std::string commandString = "nmcli c up " + droneName;
    const char *command = commandString.c_str();
    system(command);
    std::shared_ptr<ctello::Tello> drone = std::make_shared<ctello::Tello>();
    while (!drone->SendCommandWithResponse("streamon"));
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string droneYamlPathAruco = data["DroneYamlPathAruco"];
    std::string mapPath = data["loadMapPath"];
    bool withPlot = data["WithPlot"];
    bool runSimulator = data["runSimulator"];
    bool loadMap = data["loadMap"];
    bool saveMap = data["saveMap"];
    int sizeOfFrameStack = data["sizeOfFrameStack"];
    int maxForwardForSimulator = data["maxForwardForSimulator"];
    int sizeOfForwardStepSimulator = data["sizeOfForwardStepSimulator"];

    AutonomousDrone autonomousDrone(drone, vocPath, droneYamlPathSlam, droneYamlPathAruco, droneName,
                                    loadMap, mapPath, saveMap, sizeOfFrameStack, withPlot);
    if (runSimulator) {
        autonomousDrone.runSimulator(maxForwardForSimulator, sizeOfForwardStepSimulator);
    } else {
        autonomousDrone.run();
    }
    return 1;
}