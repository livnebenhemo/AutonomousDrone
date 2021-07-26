//
// Created by rbdstudent on 26/05/2021.
//
// #define _GLIBCXX_USE_CXX11_ABI 0/1
#include<iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include "ctello.h"
#include <opencv2/aruco.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/core/persistence.hpp>
#include <cstdio>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <thread>
//#include <wiringPi.h>


class Charger {
public:
    Charger(std::vector<std::pair<int, double>> markers, std::string chargerBluetoothAddress,
            std::shared_ptr<cv::VideoCapture> capture, std::shared_ptr<bool> holdCamera, std::string droneWifiName,
            std::string telloYamlFilePath, std::shared_ptr<ctello::Tello> drone, std::shared_ptr<cv::Mat> frame,
            bool withImShow = false,
            int raspberryToTelloPinNumber = 26,
            double slowSpeedDistance = 0.15,
            double fastSpeedDistance = 1.0,
            double distanceFromWall = 0.78,
            double distanceToWall = 0.76,
            double distanceUpDownMarker = 0.1,
            double distanceRightFromArucoCenter = 0.03,
            double distanceLeftFromArucoCenter = 0.0, double almostStopSpeedDistance = 0.02);

    void chargeByPaper();
    static std::vector<cv::Mat> getCameraCalibration(std::string path);

    bool run();

private:
    //marker id as a key and marker size as value, 0.01 is 1 cm
    std::vector<std::pair<int, double>> markers;
    std::string chargerBluetoothAddress;
    // 0.01 is 1 cm
    int raspberryToTelloPinNumber;
    double almostStopSpeedDistance;
    double slowSpeedDistance;
    double fastSpeedDistance;
    std::shared_ptr<bool> holdCamera;
    double distanceFromWall;
    double distanceToWall;
    double distanceUpDownMarker;
    double distanceRightFromArucoCenter;
    double distanceLeftFromArucoCenter;
    std::shared_ptr<cv::VideoCapture> capture;
    bool stop = false;
    std::string droneWifiName;
    bool stopCameraThread = false;
    std::shared_ptr<cv::Mat> frame;
    double upDown = 0.0;
    double forward = 0.0;
    double rightLeft = 0.0;
    int leftOverAngle = 0;
    std::string telloYamlFilePath;
    bool cameraOpen = false;
    int currentMarker;
    float currentMarkerSize;
    std::shared_ptr<ctello::Tello> drone;
    bool withImShow = false;


    bool manageDroneCommand(std::string command, int amountOfAttempt = 2, int amountOfSleep = 0);

    void resetTrackingGlobals();

    bool communicateWithCharger(int socket, bool closeSocket = false);

    bool chargerByEstimation();


    void getEulerAngles(cv::Mat &rotCameraMatrix, cv::Vec3d &eulerAngles);

    bool correctDroneAngle(double currentRightLeft, int currentLeftOverAngle);

    int sendMsg(int socket, std::string msg, int amountOfAttempts);

    double landDroneCarefully(float markerSize, int markerId);

    int getLeftOverAngleFromRotationVector(cv::Vec<double, 3> rvec);

    std::string getForwardSpeedText(double distance);

    std::string getBackwardsSpeedText(double distance);

    std::string getLeftSpeedText(double distance);

    std::string getRightSpeedText(double distance);

    void trackMarker();

    void getCameraFeed();

    void droneMonitor();

    std::string readMsg(int socket);

    int getChargerSocket(std::string connectionAddr);

    bool openCharger(int socket, bool closeSocket = true);

    bool closeCharger(int socket, bool closeSocket = true);

    void turnDroneOnOrOff(int sleepAmount = 20);

    bool connectToDrone(std::string droneName);


    bool communicateWithCharger(char connectionAddr[]);

};
