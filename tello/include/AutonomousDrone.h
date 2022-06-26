//
// Created by rbdstudent on 15/06/2021.
//

#ifndef TELLO_AUTONOMOUSDRONE_H
#define TELLO_AUTONOMOUSDRONE_H

#include "include/Navigation.h"
#include "include/Room.h"
#include "include/Polygon.h"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <memory>
#include "../slam/include/System.h"
#include <iostream>
#include <unistd.h>
#include "Converter.h"
#include <string>
#include <thread>
#include "Charger.h"

class AutonomousDrone {
public:
    AutonomousDrone(std::shared_ptr<ctello::Tello> drone,/*std::shared_ptr<cv::VideoCapture> capture,*/
                    std::string vocabularyFilePath, std::string cameraYamlPath, const std::string &arucoYamlPath,
                    std::string droneWifiName, int sizeOfFrameStack = 20,
                    bool withPlot = false, bool isManual=false, bool switchBattery=false,
                    std::string chargerBluetoothAddress = "3C:61:05:03:81:E2");

    void run();

    void runSimulator(int maxForwardDistance, int forwardAmount);

private:
    bool updateCurrentFrame(ORB_SLAM2::Frame frame);

    static int
    protectiveSphere(const Point &dronePosition, const std::vector<Point> &points, double sphereRadius = 0.4,
                     double epsilon = 0.125,
                     int minSamples = 15);

    static Point
    protectiveSphereByClosePoint(Point dronePosition, std::vector<Point> points, double sphereRadius = 0.4,
                                 double epsilon = 0.125,
                                 int minSamples = 15);

    void runOrbSlam();

    void areWeInWrongScale(std::vector<Frame> &frames);

    std::vector<Point> getCurrentMap();

    bool manageDroneCommand(const std::string &command, int amountOfAttempt = 3, int amountOfSleep = 0);

    bool doTriangulation();

    void alertLowBattery();

    void stayInTheAir();

    void disconnectDrone();

    void getCameraFeed();

    void howToRotate(int angle, bool clockwise, bool buildMap = false);

    void rotateDrone(int angle, bool clockwise, bool buildMap = false);

    bool checkIfPointInFront(const Point &point, int minSamples = 15, double eps = 0.125);

    void beginScan(bool findHome = false, int rotationAngle = 25);

    void getNavigationPoints(bool isExit = false);

    std::pair<int, bool> getRotationToFrameAngle(const Point &point);

    std::pair<Point, Point> getNavigationVector(const Point &previousPosition, const Point &destination);

    void collisionDetector(const Point &mdestination);

    void monitorDroneProgress(const Point &destination);

    void maintainAngleToPoint(const Point &destination, bool rotateToFrameAngle = true);

    std::tuple<int, bool, int>
    checkMotion(const Point &oldestPosition, const Point &currentPosition, const Point &closePoint,
                int angleRange = 90) const;

    void connectDrone(bool isReconnect = false);

    void saveMap(int fileNumber = 0);

    static Point convertFrameToPoint(Frame &frame);

    void updateCurrentLocation(const cv::Mat &Tcw);

    void flyToNavigationPoints();

    double distanceToHome();

    bool navigateDrone(const Point &destination, bool rotateToFrameAngle = true);

    void goUpOrDown(const Point &destination);

    bool findAndGoHome(int howClose, bool stopNavigation = false);

    void exitRoom();

    enum drone_modes {
        scanning, navigation, noBattery
    };
    std::string vocFilePath;
    Point navigationDestination;
    int dronePort = 8000;
    std::string yamlFilePath;
    Point currentLocation;
    ORB_SLAM2::System *orbSlamPointer;
    drone_modes current_drone_mode = scanning;
    std::shared_ptr<ctello::Tello> drone;
    std::shared_ptr<cv::Mat> currentImage;
    std::shared_ptr<cv::VideoCapture> capture;
    std::vector<cv::Mat> cameraParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::shared_ptr<bool> holdCamera;
    std::string droneWifiName;
    bool withPlot = false;
    Point home;
    std::string arucoYamlPath;
    bool stop = false;
    bool orbSlamRunning = false;
    bool runCamera = true;
    bool printSomething = false;
    bool rerunCamera = false;
    bool localized = false;
    bool canStart = false;
    bool isExit = true;
    bool commandingDrone = false;
    bool statusingDrone = false;
    int forwardAdvance = 40;
    int speed = 20;
    const char *const TELLO_STREAM_URL{"udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=5000"};
    std::vector<Room> rooms;
    std::unordered_map<int, int> whereWeLostLocalization;
    Room currentRoom;
    std::string chargerBluetoothAddress;
    bool droneRotate = false;
    bool isBlocked = false;
    bool lookingBack = false;
    bool rightSideBlocked = false;
    bool isMinusUp = true;
    bool leftSideBlocked = false;
    bool loopCloserHappened = false;
    double desiredAngle = 0.0;
    bool lowBattery = false;
    bool droneNotFly = false;
    bool isManual = false;
    bool useCharger = true;
    int maxRotationAngle = 25;
    bool reachedCheckpoint = false;
    bool gettingFurther = false;
    bool gettingCloser = false;
    Frame currentFrame;
    std::vector<std::pair<int, double>> markers;
    std::vector<Frame> lastFrames;
    int sizeOfFrameStack;
    double closeThreshold = 0.4;
    bool isTurning = false;
    double safetyThreshold = 0.01;
    bool exitStayInTheAirLoop;
    bool weInAWrongScale;

    double colorDetection();

    bool manageAngleDroneCommand(int angle, bool clockwise, int amountOfAttempt, int amountOfSleep);


    void buildSimulatorMap(int rotationAngle);

    void exhaustiveGlobalAdjustment();

    bool exhaustiveGlobalAdjustmentInProgress = false;

    void takeOffWithLocalization();

    void manuallyFlyToNavigationPoints();

    bool manuallyNavigateDrone(const Point &destination, bool rotateToFrameAngle = true);

    void moveWithKeyboard();

    void switchBattery(int switchingTime = 25);

    void flyToNavigationPointsNoRRT();
};

#endif //TELLO_AUTONOMOUSDRONE_H
