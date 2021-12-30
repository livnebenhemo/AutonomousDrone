//
// Created by rbdstudent on 30/06/2021.
//

#include <ctello.h>

int main() {
    ctello::Tello drone(true);
    while (!drone.SendCommandWithResponse("streamon"));
    std::ofstream batteryFile;
    batteryFile.open("/tmp/batteryFile.txt", std::ios_base::app);
    while (true){
        drone.SendCommand("rc 0 0 0 0");
        sleep(2);
        int battery = drone.GetBattery();
        std::cout << battery <<std::endl;
        batteryFile << battery << std::endl;
    }

}
