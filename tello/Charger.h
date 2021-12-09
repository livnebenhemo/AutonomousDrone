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
#include "Auxiliary.h"

class Charger {
public:
    Charger(std::vector<std::pair<int, double>> markers, std::shared_ptr<bool> holdCamera, std::string droneWifiName,
            std::string telloYamlFilePath, std::shared_ptr<ctello::Tello> drone, std::shared_ptr<cv::Mat> frame,
            int currentPort,
            bool withImShow = false,
            std::string chargerBluetoothAddress = "3C:61:05:03:81:E2",
            int raspberryToTelloPinNumber = 26,
            double slowSpeedDistance = 0.15,
            double fastSpeedDistance = 1.0,
            double distanceFromWall = 0.80,
            double distanceToWall = 0.77,
            double distanceUpDownMarker = 0.2,
            double distanceRightFromArucoCenter = -0.02,
            double distanceLeftFromArucoCenter = -0.05, double almostStopSpeedDistance = 0.02);

    void chargeByPaper();

    void travelTo3Points();

    static std::vector<cv::Mat> getCameraCalibration(const std::string& path);

    void static getEulerAngles(cv::Mat &rotCameraMatrix, cv::Vec3d &eulerAngles);

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
    int currentPort;
    std::shared_ptr<cv::VideoCapture> capture;
    bool stop = false;
    std::string droneWifiName;
    bool stopCameraThread = false;
    std::shared_ptr<cv::Mat> frame;
    double upDown = 0.0;
    double forward = 0.0;
    double rightLeft = 0.0;
    std::pair<int, bool> leftOverAngle{0, false};
    std::string telloYamlFilePath;
    bool cameraOpen = false;
    int currentMarker;
    float currentMarkerSize;
    double leftRightError;
    double forwardBackwardError;
    double upDownError;
    double amountOfUSleepForDroneRcCommand = 300000;
    double amountOfUSleepForTrackMarker = 50000;
    int amountOfMeasurement = floor(amountOfUSleepForDroneRcCommand / amountOfUSleepForTrackMarker);

    std::shared_ptr<ctello::Tello> drone;
    bool withImShow = false;

    void
    calculateAxisErrors(double rightLeftExpectation, double forwardBackwardsExpectation, double upDownExpectation);

    bool manageDroneCommand(const std::string& command, int amountOfAttempt = 2, int amountOfSleep = 0);

    void resetTrackingGlobals();

    static bool communicateWithCharger(int socket, bool closeSocket = false);

    bool chargeByEstimation(int batteryAtStart);

    void monitorDroneState();

    std::string getMovementInDepth(double forwardBackwards);

    bool correctDroneAngle(std::pair<int, bool> currentLeftOverAngle);

    static int sendMsg(int socket, const std::string& msg, int amountOfAttempts);

    double navigateToMarker(float markerSize, int markerId);

    double navigateToMarkerByExpectation(float markerSize, int markerId);

    static std::pair<int, bool> getLeftOverAngleFromRotationVector(const cv::Vec<double, 3>& rvec);

    std::string getForwardSpeedText(double distance) const;

    std::string getBackwardsSpeedText(double distance) const;

    std::string getLeftSpeedText(double distance) const;

    std::string getRightSpeedText(double distance) const;

    void trackMarker();

    void getCameraFeed();

    void droneMonitor();

    static std::string readMsg(int socket);

    static int getChargerSocket(std::string connectionAddr);

    static bool openCharger(int socket, bool closeSocket = true);

    static bool closeCharger(int socket, bool closeSocket = true);

    static void turnDroneOnOrOff(int sleepAmount = 20);

    static bool connectToDrone(const std::string& droneName);


    bool communicateWithCharger(char connectionAddr[]);

};
