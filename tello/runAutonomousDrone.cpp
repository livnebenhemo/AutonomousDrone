//
// Created by rbdstudent on 30/06/2021.
//
#include "AutonomousDrone.h"

int main() {
    std::shared_ptr<ctello::Tello> drone = std::make_shared<ctello::Tello>();
    drone->Bind();

    while (!drone->SendCommandWithResponse("streamon"));

    AutonomousDrone autonomousDrone(drone, "/home/rbdstudent/ORB_SLAM2/ORB_SLAM2/Vocabulary/ORBvoc.txt",
                                    "/home/rbdstudent/ORB_SLAM2/ORB_SLAM2/Examples/Monocular/tello_5BFA7F.yaml",
                                    "/home/rbdstudent/ORB_SLAM2/ORB_SLAM2/Examples/Monocular/tello_aruco_5BFA7F.yaml",
                                    "TELLO-5BFA7F", 20, false);
    autonomousDrone.run();

}
