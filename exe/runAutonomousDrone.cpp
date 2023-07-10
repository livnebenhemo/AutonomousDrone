//
// Created by rbdstudent on 30/06/2021.
//
#include "include/AutonomousDrone.h"
#include <nlohmann/json.hpp>

int main(int argc, char** argv) {
    bool isManual = false;
    bool switchBattery = false;
    if (argc >= 2){
        if (strcmp(argv[1],"manual") == 0) {
            isManual = true;
        }
        if (argc == 3){
            if (strcmp(argv[2],"switchBattery") == 0) {
                switchBattery = true;
            }
        }
    }
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
    std::string loadMapCSV = data["loadMapCSV"];
    bool saveMap = data["saveMap"];
    int sizeOfFrameStack = data["sizeOfFrameStack"];
    int maxForwardForSimulator = data["maxForwardForSimulator"];
    int sizeOfForwardStepSimulator = data["sizeOfForwardStepSimulator"];
    std::string saveMapPath = data["saveMapPath"];
    std::string saveMapPathCSV = data["saveMapPathCSV"];
    int coreset_size = data["coreset_size"];
    std::string python_file_path = data["python_file_path"];


    AutonomousDrone autonomousDrone(drone, vocPath, droneYamlPathSlam,
                                    droneYamlPathAruco, droneName, loadMap, loadMapCSV,
                                    mapPath, saveMap, saveMapPath, saveMapPathCSV, sizeOfFrameStack, withPlot,
                                    isManual, switchBattery, coreset_size, python_file_path);
    if (runSimulator) {
        autonomousDrone.runSimulator(maxForwardForSimulator, sizeOfForwardStepSimulator);
    } else {
        autonomousDrone.run();
    }
    return 1;
}
