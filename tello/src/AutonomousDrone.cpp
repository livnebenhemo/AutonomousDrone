#pragma clang diagnostic push
#pragma ide diagnostic ignored "ArgumentSelectionDefects"
//
// Created by rbdstudent on 15/06/2021.
//

#include "include/AutonomousDrone.h"

#include <utility>


bool FirstCopy = true;
bool runDrone = true;
double relativeChange = 0;
double lastRelativeChange = 0;


AutonomousDrone::AutonomousDrone(std::shared_ptr<ctello::Tello> drone,
                                 std::string vocabularyFilePath,
                                 std::string cameraYamlPath,
                                 const std::string &arucoYamlPath,
                                 std::string droneWifiName,
                                 bool loadMap, std::string &loadMapCSV,
                                 std::string &mapPath,
                                 bool saveMap,
				                 std::string& saveMapPath,
                                 std::string& saveMapPathCSV,
                                 int sizeOfFrameStack,
                                 bool withPlot,
                                 bool isManual,
                                 bool switchBattery,
                                 std::string chargerBluetoothAddress) {
    currentFrame = Frame();
    home = Point();
    this->drone = std::move(drone);
    this->vocFilePath = std::move(vocabularyFilePath);
    this->yamlFilePath = std::move(cameraYamlPath);
    this->withPlot = withPlot;
    weInAWrongScale = false;
    this->sizeOfFrameStack = sizeOfFrameStack;
    rooms = std::vector<Room>{};
    this->orbSlamPointer = nullptr;
    exitStayInTheAirLoop = false;
    //this->capture = capture;
    holdCamera = std::make_shared<bool>(false);
    lastFrames = std::vector<Frame>{};
    currentImage = std::make_shared<cv::Mat>();
    this->capture = std::make_shared<cv::VideoCapture>(TELLO_STREAM_URL, cv::CAP_FFMPEG);
    this->chargerBluetoothAddress = std::move(chargerBluetoothAddress);
    this->arucoYamlPath = arucoYamlPath;
    this->droneWifiName = std::move(droneWifiName);
    this->isManual = isManual;
    this->useCharger = !switchBattery;
    markers.emplace_back(std::pair<int, double>(1, 0.07));
    cameraParams = Charger::getCameraCalibration(arucoYamlPath);
    dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_6X6_250);
    this->loadMap = loadMap;
    this->loadMapCSV = loadMapCSV;
    this->mapPath = mapPath;
    this->saveBinMap = saveMap;
    this->saveMapPath = saveMapPath;
    this->saveMapPathCSV = saveMapPathCSV;
    this->navigateDroneHomePath = std::stack<std::string>();
}

void AutonomousDrone::getCameraFeed() {
    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%dT%H:%S:%MZ", gmtime(&now));
    std::cout << "outcpp" + std::string(time_buf) + ".avi" << std::endl;
    cv::VideoWriter video("/tmp/outcpp" + std::string(time_buf) + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 10,
                          cv::Size(960, 720));
    runCamera = true;
    // idea : while True
    while (true) {
        if (runCamera) {
            if (!capture->isOpened() || *holdCamera) {
                usleep(5000);
                continue;
            }
            capture->read(*currentImage);
            video.write(*currentImage);
        }
        if (printSomething)
            std::cout << std::flush;
    }
    video.release();
}

double AutonomousDrone::colorDetection() {
    if (!currentImage->empty()) {
        cv::Mat im = *currentImage;
        cv::Mat rgbChannels[3];
        cv::split(im, rgbChannels);
        cv::Mat redGreenDifferences(im.rows, im.cols, im.type());
        cv::Mat blueYellowDifferences(im.rows, im.cols, im.type());
        for (int i = 0; i < im.rows; ++i) {
            for (int j = 0; j < im.cols; ++j) {
                auto red = rgbChannels[0].at<uchar>(i, j);
                auto green = rgbChannels[1].at<uchar>(i, j);
                auto blue = rgbChannels[2].at<uchar>(i, j);
                redGreenDifferences.at<uchar>(i, j) = std::abs(red - green);
                blueYellowDifferences.at<uchar>(i, j) = std::abs((red + green) / 2 - blue);
            }
        }
        cv::Scalar redGreenMean, redGreenStdDev;
        cv::Scalar blueYellowMean, blueYellowStdDev;
        cv::meanStdDev(redGreenDifferences, redGreenMean, redGreenStdDev);
        cv::meanStdDev(blueYellowDifferences, blueYellowMean, blueYellowStdDev);
        auto stdRoot = std::sqrt(std::pow(blueYellowStdDev[0], 2) + std::pow(redGreenStdDev[0], 2));
        auto meanRoot = std::sqrt(std::pow(redGreenMean[0], 2) + std::pow(blueYellowMean[0], 2));
        double colorfulness = stdRoot + (0.3 * meanRoot);
        std::cout << "colorfulness: " << colorfulness << std::endl;
        return colorfulness;
    }
    return 0;
}

void AutonomousDrone::runOrbSlam() {
    std::thread getCameraThread(&AutonomousDrone::getCameraFeed, this);
    orbSlamRunning = true;
    ORB_SLAM2::System SLAM(vocFilePath, yamlFilePath, ORB_SLAM2::System::MONOCULAR, true, loadMap, mapPath, false);
    orbSlamPointer = &SLAM;
    double timeStamp = 0.2;
    int amountOfChanges = 0;
    canStart = true;
    while (orbSlamRunning) {
        if (!currentImage->empty()) {
            while (droneRotate) {
                usleep(20000); // 1/25 fps
            }
            cv::Mat img = currentImage->clone();
            cv::Mat orbSlamCurrentPose = orbSlamPointer->TrackMonocular(img, timeStamp);
            if (orbSlamCurrentPose.empty()) {
                localized = false;
                continue;
            }
            localized = true;
            updateCurrentLocation(orbSlamCurrentPose);
            auto orbSlamTracker = orbSlamPointer->GetTracker();
            if (orbSlamTracker) {
                auto mvpMapPoints = orbSlamTracker->mCurrentFrame.GetMvpMapPoints();
                if (!mvpMapPoints.empty()) {
                    //updateCurrentFrame(orbSlamTracker->mCurrentFrame);
                    if (orbSlamTracker->mCurrentFrame.getFrameId() != currentFrame.frameId) {
                        int frameId = orbSlamTracker->mCurrentFrame.getFrameId();
                        std::vector<Point> framePoints;
                        for (auto &p: orbSlamTracker->mCurrentFrame.GetMvpMapPoints()) {
                            if (p.second && !p.second->isBad()) {
                                Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(p.second->GetWorldPos());
                                framePoints.emplace_back(
                                        Point(v.x(), v.z(), v.y(), currentLocation.rotationMatrix, frameId));
                            }
                        }
                        currentFrame = Frame(currentLocation.x, currentLocation.y, currentLocation.z,
                                             currentLocation.rotationMatrix, frameId,
                                             framePoints, framePoints.size());


                        lastFrames.insert(lastFrames.begin(), currentFrame);
                        if (lastFrames.size() > sizeOfFrameStack) {
                            lastFrames.pop_back();
                        }
                    }
                    int numberOfChanges = orbSlamPointer->GetMap() ? orbSlamPointer->GetMap()->GetLastBigChangeIdx()
                                                                   : 0;
                    /*if (amountOfChanges != numberOfChanges) {
                        loopCloserHappened = true;
                        amountOfChanges = numberOfChanges;
                    }*/
                }
            }

        }
    }
    std::cout << "shuting orbslam down " << std::endl;
    orbSlamPointer->Shutdown();
    cvDestroyAllWindows();
}

void AutonomousDrone::updateCurrentLocation(const cv::Mat &Tcw) {
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat twc = -Rwc.t() * Tcw.rowRange(0, 3).col(3);
    currentLocation = Point(twc.at<float>(0), twc.at<float>(2), twc.at<float>(1), Rwc);
}

bool AutonomousDrone::updateCurrentFrame(ORB_SLAM2::Frame frame) {
    if (frame.getFrameId() != currentFrame.frameId) {
        int frameId = frame.getFrameId();
        std::vector<Point> framePoints;
        for (auto &p: frame.GetMvpMapPoints()) {
            if (p.second && !p.second->isBad()) {
                Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(p.second->GetWorldPos());
                framePoints.emplace_back(
                        Point(v.x(), v.z(), v.y(), currentLocation.rotationMatrix, frameId));
            }
        }
        currentFrame = Frame(currentLocation.x, currentLocation.y, currentLocation.z,
                             currentLocation.rotationMatrix, frameId,
                             framePoints, framePoints.size());


        lastFrames.insert(lastFrames.begin(), currentFrame);
        if (lastFrames.size() > sizeOfFrameStack) {
            lastFrames.pop_back();
        }
    }
}


std::vector<Point> getPointsFromFile(const std::string& fileName) {
    std::vector<Point> points;
    std::ifstream myFile(fileName);
    std::string line;
    while (std::getline(myFile, line)) {
        std::stringstream lineStream(line);
        Point point;
        lineStream >> point.x;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.z;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.y;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.frameId;
        points.push_back(point);
    }
    return points;
}


std::vector<Point> AutonomousDrone::getCurrentMap() {
    std::vector<Point> currentMap;
    if (!loadMap) {
        for (auto p: orbSlamPointer->GetMap()->GetAllMapPoints()) {
            if (p && !p->isBad()) {
                auto frame = p->GetReferenceKeyFrame();
                if (!frame) {  // TODO : check it !!!!!!!
                    continue;
                }
                int frameId = frame->mnFrameId;
                cv::Mat Tcw = frame->GetPose();
                auto point = p->GetWorldPos();
                cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
                Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
                currentMap.emplace_back(Point(v.x(), v.z(), v.y(), Rwc, frameId));
            }
        }
    }
    saveMap(rooms.size());
    if (loadMap)
        currentMap = getPointsFromFile("/tmp/pointDataExtended.csv");
    return currentMap;
}


void AutonomousDrone::saveMap(int fileNumber) {
    while (true) {
        try {
            auto orbSlamMap = orbSlamPointer->GetMap();
            if (orbSlamMap) {
                std::ofstream pointData;
                if (loadMap) {
                    pointData.open("/tmp/pointDataExtended.csv");
                    if (FirstCopy) { // copy the base map (first scan)
                        pointData.clear();
                        std::cout << "Copy base map" << std::endl;
                        FirstCopy = false;
                        std::ifstream sourcePointData(loadMapCSV);
                        char ch;
                        while(sourcePointData && sourcePointData.get(ch) ) {
                            pointData.put(ch);
                        }
                        sourcePointData.close();
                    }
                }
                else
                    pointData.open("/tmp/pointData" + std::to_string(fileNumber) + ".csv");
                for (auto p: orbSlamMap->GetAllMapPoints()) {
                    if (p && !p->isBad()) {
                        auto frame = p->GetReferenceKeyFrame();
                        if (!frame){
                            continue;
                        }
                        int frameId = frame->mnFrameId;
                        cv::Mat Tcw = frame->GetPose();
                        auto point = p->GetWorldPos();
                        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3);
                        cv::Mat twc = -Rwc.t() * Tcw.rowRange(0, 3).col(3);
                        Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
                        pointData << v.x() << "," << v.y() << "," << v.z() << "," << Rwc.at<float>(0, 0) << ','
                                  << Rwc.at<float>(0, 1) << ',' << Rwc.at<float>(0, 2)
                                  << ',' << Rwc.at<float>(1, 0) << ',' << Rwc.at<float>(1, 1) << ','
                                  << Rwc.at<float>(1, 2) << ','
                                  << Rwc.at<float>(2, 0)
                                  << ',' << Rwc.at<float>(2, 1) << ',' << Rwc.at<float>(2, 2) << "," << frameId <<
                                  "," << twc.at<float>(0) << "," << twc.at<float>(1) << "," << twc.at<float>(2)
                                  << std::endl;
                    }
                }
                pointData.close();
                std::cout << "saved map" << std::endl;
                break;
            }
            usleep(100);
        } catch (std::exception &exception) {
            std::cout << "something happend at save map:" << exception.what() << std::endl;
        }
        catch (...) {
            std::cout << "something happend at save map" << std::endl;
        }
    }
}

bool AutonomousDrone::manageDroneCommand(const std::string &command, int amountOfAttempt, int amountOfSleep) {
    while (orbSlamPointer->GetLocalMapping()->isStopped()) {
        usleep(100);
    }
    while (commandingDrone || statusingDrone) {
        usleep(200);
    }

    while (amountOfAttempt--) {
        if (!droneNotFly) {
            commandingDrone = true;
            if (drone->SendCommandWithResponse(command, 10000)) {
                commandingDrone = false;
                if (amountOfSleep) {
                    sleep(amountOfSleep);
                } else {
                    usleep(100000);
                }
                return true;
            } else {
                commandingDrone = false;
                sleep(1);
            }
        }
    }
    sleep(1);
    commandingDrone = false;
    return false;
}

bool
AutonomousDrone::manageAngleDroneCommand(int angle, bool clockwise, int amountOfAttempt = 3, int amountOfSleep = 0) {
    droneRotate = true;
    bool response = false;
    if (clockwise) {
        response = manageDroneCommand("cw " + std::to_string(angle), amountOfAttempt, 0);
    } else {
        response = manageDroneCommand("ccw " + std::to_string(angle), amountOfAttempt, 0);
    }
    droneRotate = false;
    sleep(amountOfSleep);
    return response;
}

bool AutonomousDrone::doTriangulation() {
    bool a = manageDroneCommand("forward 30", 5, 3);
    bool b = manageDroneCommand("back 30", 5, 3);
    return a&&b;
}


bool AutonomousDrone::doTriangulationUpDown() {
    bool a = manageDroneCommand("down 30", 5, 3);
    bool b = manageDroneCommand("up 30", 5, 3);
    return a&&b;
}

void AutonomousDrone::rotateDrone(int angle, bool clockwise, bool buildMap, bool isHome) {
    if (!lowBattery && angle >= 3) {
        if (localized) {
            while (!manageAngleDroneCommand(angle, clockwise, 10, 2)) {
                usleep(300000);
            }
            if (!isHome){
                if (clockwise)
                    this->navigateDroneHomePath.push("ccw " + std::to_string(angle));
                else
                    this->navigateDroneHomePath.push("cw "  + std::to_string(angle));
            }
        }
        int amountOfLostLocalizations = 0;
        if (!localized  && !stop) {
            int regainLocalizationAngle = angle;
            manageDroneCommand("back 20", 1, 1);
            if (!isHome)
                this->navigateDroneHomePath.push("forward 20");
            while (!localized && !lowBattery) {
                // manageDroneCommand("back 20", 1, 1);
                sleep(3);
                manageAngleDroneCommand(angle, !clockwise, 3, 5);
                if (!localized) {
                    doTriangulation();
                }
                regainLocalizationAngle = maxRotationAngle;
                amountOfLostLocalizations += 1;
                /*if (amountOfLostLocalizations == 3) {
                    manageDroneCommand("back 30", 3, 1);
                }*/
            }
        }
        if (buildMap) {
            doTriangulation();
        }
        if (amountOfLostLocalizations) {
            /*if (current_drone_mode == navigation) {
                auto [angleAfterLost, clockwiseAfterLost] = getRotationToFrameAngle(navigationDestination);
                howToRotate(angleAfterLost, clockwiseAfterLost, false);
            } else {
                amountOfLostLocalizations %= int(360 / maxRotationAngle);
                while (amountOfLostLocalizations - 1) {
                    rotateDrone(maxRotationAngle, clockwise, false);
                    amountOfLostLocalizations--;
                }
                rotateDrone(angle, clockwise, true);
            }*/
            amountOfLostLocalizations %= int(360 / maxRotationAngle);
            while (amountOfLostLocalizations - 1) {
                rotateDrone(maxRotationAngle, clockwise, false);
                amountOfLostLocalizations--;
            }
            rotateDrone(angle, clockwise, true);
        }
    }
}

void AutonomousDrone::howToRotate(int angle, bool clockwise, bool buildMap, bool isHome) {
    if (angle < 360) {
        if (angle < 30) {
            rotateDrone(angle, clockwise, buildMap, isHome);
        } else {
            int amountOfRotation = ceil(angle / maxRotationAngle);
            while (amountOfRotation--) {
                if (!lowBattery) {
                    rotateDrone(maxRotationAngle, clockwise, buildMap, isHome);
                } else {
                    switchBattery();
                    // return;
                }
            }
            rotateDrone(int(angle % maxRotationAngle), clockwise, buildMap, isHome);
        }
    }
}

void AutonomousDrone::disconnectDrone() {
    manageDroneCommand("land", 3, 5);
    manageDroneCommand("streamoff", 3, 3);
}

void AutonomousDrone::alertLowBattery() {
    lowBattery = false;
    int battery;
    int amountOfAttempts = 3;
    while (true) {
        std::cout << std::flush;
        if (!printSomething) { // drone is connected
            battery = drone->GetBatteryStatus();
            while (true) {
                if (!commandingDrone) {
                    statusingDrone = true;
                    battery = drone->GetBatteryStatus();
                    statusingDrone = false;
                    std::cout << "battery:" << battery << std::endl;
                    break;
                } else {
                    usleep(300000);
                }
            }
            if (battery < 20) {
                if (!(amountOfAttempts--)) {
                    if (current_drone_mode == navigation && navigationDestination == Point()) {
                        current_drone_mode = noBattery;
                    } else {
                        lowBattery = true;
                        holdMonitorThread = true;
                        stop = true;
                    }
                    std::cout << "low battery" << std::endl;
                    continue;
                    // break;
                } else {
                    sleep(1);
                }
            } else {
                amountOfAttempts = 3;
            }
            sleep(5);
        }
    }
}

bool AutonomousDrone::checkIfPointInFront(const Point &point, int minSamples, double eps) {
    int amountOfClosePoints = 0;
    for (const auto &framePoint: currentFrame.points) {
        amountOfClosePoints += sqrt(pow(point.z - framePoint.z, 2) + pow(point.x - framePoint.x, 2)) < eps ? 1 : 0;
        if (amountOfClosePoints == minSamples) {
            std::cout << "point infront" << std::endl;
            return true;
        }
    }
    return false;
}

double AutonomousDrone::distanceToHome() {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(*currentImage, dictionary, corners, ids);
    if (!ids.empty()) {
        bool isZeroId = false;
        for (auto id: ids) {
            std::cout << "we found:" << id << std::endl;
            if (!id || isZeroId) {
                isZeroId = true;
                break;
            }
            for (auto marker: markers) {
                if (id == marker.first) {
                    isZeroId = false;
                    break;
                }
                isZeroId = true;
            }
        }
        if (!isZeroId) {
            std::vector<cv::Vec3d> localRvecs, localTvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.07, cameraParams[0], cameraParams[1],
                                                 localRvecs,
                                                 localTvecs);
            cv::Mat rmat = cv::Mat::eye(3, 3, CV_64FC1);
            try {
                cv::Rodrigues(localRvecs[0], rmat);
            } catch (...) {
                return 0.0;
            }
            auto t = cv::Mat(-rmat.t() * cv::Mat(localTvecs[0]));
            cv::Vec3d eulerAngles;
            Charger::getEulerAngles(rmat, eulerAngles);
            int currentLeftOverAngle = eulerAngles[0];
            if (currentLeftOverAngle > 0) {
                if (currentLeftOverAngle > 130 && currentLeftOverAngle < 165)
                    return manageAngleDroneCommand(180 - currentLeftOverAngle, true, 3, 2);
                else if (currentLeftOverAngle > 10 && currentLeftOverAngle < 40)
                    return manageAngleDroneCommand(currentLeftOverAngle, false, 3, 2);
            } else {
                currentLeftOverAngle =
                        currentLeftOverAngle < -90 ? -1 * (-180 - currentLeftOverAngle) : -1 * currentLeftOverAngle;
                if (currentLeftOverAngle > 10) {
                    if (t.at<double>(0) < 0)
                        return manageAngleDroneCommand(currentLeftOverAngle, false, 3, 2);
                    else
                        return manageAngleDroneCommand(currentLeftOverAngle, true, 3, 2);
                }

            }
            return t.at<double>(2);
        }
    }
    return 0;
}

bool AutonomousDrone::findAndGoHome(int howClose, bool stopNavigation) {
    double distance = distanceToHome();
    if (distance != 0.0) {
        std::cout << "distance home:" << distance << std::endl;
        stop = stopNavigation;
        int forward = int((distance - 1) * howClose);
        for (int i = 0; i < forward / 30; i++) {
            manageDroneCommand("forward 30");
        }
        if (forward % 30 > 20) {
            manageDroneCommand("forward " + std::to_string(int(forward % 30)), 3);
        }
        return true;
    }
    return false;
}

void AutonomousDrone::exhaustiveGlobalAdjustment() {
    exhaustiveGlobalAdjustmentInProgress = true;
    std::cout << "Start Global Bundle Adjustment: " << std::endl;
    for (auto &keyFrame: orbSlamPointer->GetMap()->GetAllKeyFrames()) {
        if (keyFrame->mnId != 0) {
            orbSlamPointer->GetLoopClosing()->RunGlobalBundleAdjustment(keyFrame->mnId);
        }
    }
    std::cout << "Finish Global Bundle Adjustment: " << std::endl;
    exhaustiveGlobalAdjustmentInProgress = false;
}

void AutonomousDrone::buildSimulatorMap(int rotationAngle) {
    current_drone_mode = scanning;
    while (!localized) {
        doTriangulation();
        if (localized) {
            break;
        }
        manageAngleDroneCommand(maxRotationAngle, false, 5, 2);
        //startAngle -= maxRotationAngle;
    }
    rotationAngle = maxRotationAngle < rotationAngle ? maxRotationAngle : rotationAngle;
    int amountOfRotations = std::floor((360/* - startAngle*/) / rotationAngle);
    for (int i = 0; i < amountOfRotations; i++) {
        howToRotate(rotationAngle, true, true);
        auto [endYaw, endClockwise] = AutonomousDrone::getRotationToFrameAngle(home);
        endYaw = 180 - std::abs(endYaw);
        endClockwise = !endClockwise;
        std::cout << "angle: " << endYaw << " clockwise: " << endClockwise << std::endl;
    }
    howToRotate((360/* - startAngle*/) % rotationAngle, true, true);
    auto [endYaw, endClockwise] = AutonomousDrone::getRotationToFrameAngle(home);
    endYaw = 180 - std::abs(endYaw);
    endClockwise = !endClockwise;
    std::cout << "angle: " << endYaw << " clockwise: " << endClockwise << std::endl;
    howToRotate(endYaw, endClockwise);
    std::thread exhaustiveGlobalAdjustmentThread(&AutonomousDrone::exhaustiveGlobalAdjustment, this);
    exhaustiveGlobalAdjustmentInProgress = true;
    while (exhaustiveGlobalAdjustmentInProgress) {
        doTriangulation();
    }
    exhaustiveGlobalAdjustmentThread.join();
}

void AutonomousDrone::beginScan(bool findHome, int rotationAngle) {
    current_drone_mode = scanning;
    manageDroneCommand("forward 30", 5, 3);
    manageDroneCommand("back 30", 5, 3);
    std::cout << "Find localization" << std::endl;
    while (!localized) {
        doTriangulation();
        if (localized) {
            break;
        }
        doTriangulationUpDown();
        if (lowBattery){
            switchBattery();
            lowBattery = false;
        }
        manageAngleDroneCommand(maxRotationAngle, true, 5, 2);
    }
    if (!loadMap) {
        std::cout << "starting scan" << std::endl;
        for (int i = 0; i < std::ceil(360 / rotationAngle) + 1; i++) {
            if (lowBattery) {
                switchBattery();
                lowBattery = false;
            }
            /*if (findHome) {
                if (findAndGoHome(90, false)) {
                    home = currentLocation;
                    orbSlamPointer->GetMapDrawer()->SetCharger(home);
                    findHome = false;
                }
            }*/
            howToRotate(rotationAngle, true, true);
            std::cout << "we did: " << (i + 1) * rotationAngle << std::endl;
        }

        std::cout << "starting global bundle Adjustments" << std::endl;
        std::thread exhaustiveGlobalAdjustmentThread(&AutonomousDrone::exhaustiveGlobalAdjustment, this);
        exhaustiveGlobalAdjustmentInProgress = true;
        exhaustiveGlobalAdjustmentThread.join();
        if (lowBattery) {
            switchBattery();
            lowBattery = false;
        }
    }

    auto prevLocation = currentLocation;
    manageDroneCommand("up 40", 3, 3);
    isMinusUp = prevLocation.z > currentLocation.z;
    manageDroneCommand("down 40", 3);
    if (!loadMap)
        orbSlamPointer->GetLoopClosing()->RunGlobalBundleAdjustment(orbSlamPointer->GetMap()->GetMaxKFid());

    if (saveBinMap) {
        char time_buf[21];
        time_t time_tNow;
        std::time(&time_tNow);
        std::strftime(time_buf, 21, "%Y%m%d%H%S%M", gmtime(&time_tNow));
        std::string currentTime(time_buf);
        std::string filename = saveMapPath;
        *holdCamera = true;
        sleep(1);
        orbSlamPointer->SaveMap(filename);
        *holdCamera = false;
    }
}

Point AutonomousDrone::convertFrameToPoint(Frame &frame) {
    return {frame.x, frame.y, frame.z, frame.rotationMatrix, frame.frameId};
}

// true - need to stop, false - continue
bool AutonomousDrone::stopCondition(const std::vector<Point>& navigationPoints){  // TODO : finish it
    for (int i = 0; i < navigationPoints.size(); ++i) {
        if (Auxiliary::calculateDistanceXY(navigationPoints[i], currentLocation) > 0.4) {  // TODO : not hard-coded?
            return false;
        }
    }
    std::cout << "finish scan area, need to stop" << std::endl;
    return true;
}


void AutonomousDrone::getNavigationPoints(bool isExit) {
    std::cout << "getting navigation points" << std::endl;
    exitStayInTheAirLoop = false;
    currentRoom.points = std::vector<Point>{};
    currentRoom.points = getCurrentMap();
    std::cout << "we saved points" << std::endl;
    std::vector<Point> points(currentRoom.points);
    Polygon polygon(points, home == Point() ? currentLocation : home, isExit);
    std::vector<Point> exitPoints = polygon.getExitPointsByPolygon();
    std::cout << "we saved exitPoints" << std::endl;
    int pointIndex = 0;
    for (const auto &point: exitPoints) {
        for (const auto &room: rooms) {
            if (!room.visitedExitPoints.empty())
                for (const auto &visitedExitPoint: room.visitedExitPoints) {
                    if (Auxiliary::calculateDistanceXY(point, visitedExitPoint) <
                        Auxiliary::calculateDistanceXY(currentLocation, point) / 2.5) {
                        exitPoints.erase(exitPoints.begin() + pointIndex);
                    }
                }

        }
        pointIndex += 1;
    }
    std::cout << "find optimal path" << std::endl;
    Navigation navigation;
    exitPoints = navigation.findOptPath(exitPoints, currentLocation);
    if (!isManual) {
        std::cout << "amount of navigation createStar Navigationpoints before star:" << exitPoints.size() << std::endl;
        exitPoints = navigation.createStarNavigation(exitPoints, currentLocation);
    }
    currentRoom.exitPoints = exitPoints;
    exitStayInTheAirLoop = true;
    std::cout << """the polygon navigation points:" << exitPoints.size() << std::endl;
}



/*std::pair<int, bool> AutonomousDrone::getRotationToFrameAngle(const Point &point) {
    cv::Mat Rwc = currentLocation.rotationMatrix.t();
    auto angles = Auxiliary::rotationMatrixToEulerAngles(Rwc);
    auto currentYaw = angles.z;
    Point directionVector(point.x - currentLocation.x, point.y - currentLocation.y, point.z - currentLocation.z);
    double directionVectorYaw = Auxiliary::radiansToAngle(std::atan2(directionVector.z, directionVector.x));
    currentYaw = currentYaw + 90;
    double angleDifference = -1 * (directionVectorYaw - currentYaw);
    while (angleDifference > 180) {
        angleDifference -= 360;
    }
    while (angleDifference <= -180) {
        angleDifference += 360;
    }
    bool clockwise = true;
    std::cout << "before logic angle:" << angleDifference << " in direction: " << bool(angleDifference > 0)
              << std::endl;
    if (angleDifference < 0) {
        clockwise = false;
        angleDifference *= -1;
    }
    std::cout << "after logic angle:" << angleDifference << " in direction: " << clockwise << std::endl;
    return std::pair<int, bool>{int(angleDifference), clockwise};
}*/


std::pair<int, bool> AutonomousDrone::getRotationToFrameAngle(const Point &point, bool first, double relativeChange) {
    /*double timeStamp = 0.2;
    cv::Mat img = currentImage->clone();
    cv::Mat orbSlamCurrentPose = orbSlamPointer->TrackMonocular(img, timeStamp);
    localized = true;
    updateCurrentLocation(orbSlamCurrentPose);*/
    std::cout << "(Tello) [rotate_to_dest_angle] Starting" << std::endl;
    // Get angle from Rwc
    cv::Mat Rwc = currentLocation.rotationMatrix.t();
    auto angles = Auxiliary::rotationMatrixToEulerAngles(Rwc);
    float angle1 = angles.z;

    cv::Point2f vec1(point.x - currentLocation.x,
                     point.y - currentLocation.y);
    // Get angle from vec1
    float angle2 = std::atan2(vec1.y, vec1.x) * 180 / M_PI;
    if (first) {
        angle1 = (angle1 + 0);
    }
    else {
        angle1 = (angle1 + relativeChange);
    }
    std::cout << "(Tello) [rotate_to_dest_angle] current_angle: " << angle1
              << " desired_angle: " << angle2 << std::endl;

    float ang_diff = angle2 - angle1;
    ang_diff = -ang_diff;
    while (ang_diff > 180) ang_diff -= 360;
    while (ang_diff <= -180) ang_diff += 360;

    std::cout << "(Tello) [rotate_to_dest_angle] ang_diff: " << ang_diff
              << std::endl;

    if (abs(ang_diff) > 0) {
        if (abs(ang_diff) < 3)  // Drone doesn't rotate in this range
            ang_diff = 0;
        sleep(1);
        if (ang_diff > 0)
            return std::pair<int, bool>{int(ang_diff), true};
        else
            return std::pair<int, bool>{int(-ang_diff), false};
    }
}

std::pair<Point, Point>
AutonomousDrone::getNavigationVector(const Point &previousPosition, const Point &destination) {
    Point currentPosition = currentLocation;
    Point droneVector = currentPosition - previousPosition;
    Point exitVector = destination - currentPosition;
    return std::pair<Point, Point>{droneVector, exitVector};
}


std::pair<int, bool> AutonomousDrone::maintainAngleToPoint(const Point &destination, bool rotateToFrameAngle, bool first,
                                                           double relativeChange, bool isHome) {
    double totalAngleChange = 0;
    std::cout << "Computing angle to rotate" << std::endl;
    auto howToRotateToFrame = getRotationToFrameAngle(destination, first, relativeChange);
    if (rotateToFrameAngle) {
        isDroneRotate = true;
        howToRotate(howToRotateToFrame.first, howToRotateToFrame.second, isHome);
        isDroneRotate = false;
    }
    sleep(3);
    return howToRotateToFrame;
}


Point
AutonomousDrone::protectiveSphereByClosePoint(Point dronePosition, std::vector<Point> points, double sphereRadius,
                                              double epsilon,
                                              int minSamples) {
    std::sort(points.begin(), points.end(), [&dronePosition](const Point &p1, const Point &p2) {
        return Auxiliary::calculateDistanceXY(p1, dronePosition) <
               Auxiliary::calculateDistanceXY(p2, dronePosition);
    });
    for (const Point &point: points) {
        if (Auxiliary::calculateDistance3D(point, dronePosition) < sphereRadius) {
            int counter = 0;
            for (const Point &point2: points) {
                if (counter == minSamples) {
                    std::cout << "distance to close point:" << Auxiliary::calculateDistance3D(point, dronePosition)
                              << std::endl;
                    return point;
                }
                counter += Auxiliary::calculateDistance3D(point, point2) < epsilon ? 1 : 0;
            }
        }
    }
    return {};
}

int
AutonomousDrone::protectiveSphere(const Point &dronePosition, const std::vector<Point> &points, double sphereRadius,
                                  double epsilon,
                                  int minSamples) {
    int pointIndex = 0;
    for (const Point &point: points) {
        pointIndex += 1;
        if (Auxiliary::calculateDistance3D(point, dronePosition) < sphereRadius) {
            int counter = 0;
            for (const Point &point2: points) {
                if (counter == minSamples) {
                    std::cout << "distance to close point:" << Auxiliary::calculateDistance3D(point, dronePosition)
                              << std::endl;
                    return pointIndex;
                }
                counter += Auxiliary::calculateDistance3D(point, point2) < epsilon ? 1 : 0;
            }
        }
    }
    return -1;
}

std::tuple<int, bool, int>
AutonomousDrone::checkMotion(const Point &oldestPosition, const Point &currentPosition, const Point &closePoint,
                             int angleRange) const {
    Point droneMotionVector(currentPosition.x - oldestPosition.x, currentPosition.y - oldestPosition.y,
                            currentPosition.z - oldestPosition.z);
    Point droneToCheckpointVector(currentPosition.x - closePoint.x, currentPosition.y - closePoint.y,
                                  currentPosition.z - closePoint.z);
    auto directions = Auxiliary::getRotationToTargetInFront(droneMotionVector, droneToCheckpointVector);
    if (directions.first < angleRange) {
        int isUp = 0;
        if (std::abs(closePoint.z - currentPosition.z) < safetyThreshold) {
            if (isMinusUp) {
                isUp = closePoint.z < currentPosition.z ? 1 : -1;
            } else {
                isUp = closePoint.z > currentPosition.z ? 1 : -1;
            }
        }
        return std::make_tuple(directions.first, directions.second, isUp);
    }
    return std::make_tuple(0, false, 0);
}

void AutonomousDrone::areWeInWrongScale(std::vector<Frame> &frames) {
    double sumOfDistances = 0.0;
    for (auto &frame: frames) {
        sumOfDistances += Auxiliary::calculateDistanceXY(convertFrameToPoint(frame), Point());
    }
    double mean = sumOfDistances / frames.size();
    double variance = 0.0;
    for (auto &frame: frames) {
        variance += pow(Auxiliary::calculateDistanceXY(convertFrameToPoint(frame), Point()) - mean, 2);
    }
    weInAWrongScale = variance < 0.001;
}

void AutonomousDrone::collisionDetector(const Point &destination) {
    isBlocked = false;
    int amountOfOutOfScale = 4;
    sleep(3);
    while (!stop) {
        if (!droneRotate) {
            auto frames = lastFrames;
            int currentSpeed = speed;
            //areWeInWrongScale(frames);
            if (weInAWrongScale) {
                std::cout << "we are in wrong scale" << std::endl;
                if (!(amountOfOutOfScale--)) {
                    stop = true;
                }
            } else {
                amountOfOutOfScale = 4;
            }
            if (localized) {
                auto frame = frames.front();
                auto lastKnownFrame = frames[int(sizeOfFrameStack / 2)];
                //int pointIndex = protectiveSphere(convertFrameToPoint(frame), frame.points, safetyThreshold);
                auto closePoint = protectiveSphereByClosePoint(currentLocation, frame.points, safetyThreshold);
                if (/*pointIndex != -1*/ !(closePoint == Point())) {
                    std::cout << "we are blocked" << std::endl;
                    Point lastKnownFrameAsPoint = convertFrameToPoint(lastKnownFrame);
                    auto [angle, clockwise, isUp] = checkMotion(lastKnownFrameAsPoint,
                                                                currentLocation, /*frame.points[pointIndex]*/
                                                                closePoint);
                    if (angle > 0) {
                        isBlocked = true;
                        speed = 0;
                        if (!droneNotFly) {
                            drone->SendCommand("rc 0 0 0 0");
                        }
                        manageDroneCommand("back 20", 3);
                        usleep(300000);
                        auto navigationVectors = getNavigationVector(lastKnownFrameAsPoint, destination);
                        auto rotationDirections = Auxiliary::getRotationToTargetInFront(navigationVectors.first,
                                                                                        navigationVectors.second);
                        if (isUp == -1) {
                            manageDroneCommand("up 20", 3);
                        } else if (isUp == 1) {
                            manageDroneCommand("down 20", 3);
                        }
                        if (rotationDirections.second == clockwise) {
                            if (rotationDirections.first > angle) {
                                std::cout << "can avoid with navigate to destintion" << std::endl;
                                howToRotate(rotationDirections.first, rotationDirections.second);
                            } else {
                                std::cout << "same direction and blocked from sides" << std::endl;
                                if (clockwise) {
                                    manageDroneCommand("left 20", 3);
                                } else {
                                    manageDroneCommand("right 20", 3);
                                }
                            }
                        } else {
                            if (rotationDirections.first < 30) {
                                std::cout << "different direction and infront" << std::endl;
                                if (!clockwise) {
                                    manageDroneCommand("left 20", 3);
                                } else {
                                    manageDroneCommand("right 20", 3);
                                }
                            } else {
                                std::cout << "different direction but not in front" << std::endl;
                                howToRotate(rotationDirections.first, rotationDirections.second);
                            }
                        }
                    }
                    speed = currentSpeed;
                    isBlocked = false;
                    sleep(2);
                }
            } else {
                if (currentFrame.amountOfKeyPoints < 1000) {
                    manageDroneCommand("back 60", 3, 1);
                    goUpOrDown(destination);
                } else {
                    manageDroneCommand("back 30", 3, 1);
                    manageDroneCommand("forward 30", 3, 1);
                }
            }
        }
        isBlocked = false;
        usleep(500000);
    }
}

void AutonomousDrone::goUpOrDown(const Point &destination) {
    if (isMinusUp) {
        if (destination.z - closeThreshold > currentFrame.z) {
            manageDroneCommand("down 30", 3, 1);
        } else if (destination.z + closeThreshold < currentFrame.z) {
            manageDroneCommand("up 30", 3, 1);
        }
    } else {
        if (destination.z + closeThreshold < currentFrame.z) {
            manageDroneCommand("down 30", 3, 1);
        } else if (destination.z - closeThreshold > currentFrame.z) {
            manageDroneCommand("up 30", 3, 1);
        }
    }
}

void AutonomousDrone::monitorDroneProgress(const Point &destination, bool toHome) {
    double previousDistance = 10000;
    usleep(500000);
    int i = 0;
    int amountOfGettingFurther = 4;
    closeThreshold = Auxiliary::calculateDistanceXY(currentLocation, destination) / 1.5; // TODO : change 2.5
    if (toHome)
        closeThreshold /= 2;
    std::cout << "close threshold:" << closeThreshold << std::endl;
    double expectedScale = closeThreshold * 0.005;
    std::cout << "scale threshold:" << expectedScale << std::endl;
    std::vector<double> distances{};
    while (!stop || holdMonitorThread) {
        if (!lowBattery) {
            double distance = Auxiliary::calculateDistanceXY(currentLocation, destination);
            if (!(i % 2)) {
                std::cout << "distance to destination: " << distance << std::endl;
            }
            if (distance <= closeThreshold) {
                stop = true;
                currentRoom.visitedExitPoints.emplace_back(destination);
                std::cout << "reached to destination" << std::endl;
                break;
            }
            if (!droneRotate && !isBlocked) {
                if (distance <= closeThreshold * 1.2) {
                    gettingCloser = true;
                    std::cout << "getting closer" << std::endl;
                    //goUpOrDown(destination);  TODO : think if delete
                    speed = 15;
                }
            }
            previousDistance = distance;
            usleep(450000);
            i++;
        }
    }
}

bool AutonomousDrone::navigateDrone(const Point &destination, bool rotateToFrameAngle, bool isHome) {
    current_drone_mode = navigation;
    std::cout << "start navigation to:" << destination.to_string() << std::endl;
    navigationDestination = destination;
    orbSlamPointer->GetMapDrawer()->SetDestination(navigationDestination);
    stop = false;
    loopCloserHappened = false;
    weInAWrongScale = false;
    if (Auxiliary::calculateDistanceXY(destination, currentLocation) < 0.4) {  // TODO : not hard-coded?
        std::cout << "we are close to the point" << std::endl;
        return true;
    }
    //std::thread collisionDetectorThread(&AutonomousDrone::collisionDetector, this, destination);
    //std::thread maintainAngleToPointThread(&AutonomousDrone::maintainAngleToPoint, this, destination,
    //                                       rotateToFrameAngle);
    std::thread monitorDroneProgressThread(&AutonomousDrone::monitorDroneProgress, this, destination, isHome);
    bool areWeNavigatingHome = destination == home;
    std::cout << "is home : " << isHome << std::endl;
    if(!isHome) {
        this->navigateDroneHomePath = std::stack<std::string>();
        lastRelativeChange = relativeChange;
        auto howToRotateToFrame = maintainAngleToPoint(destination, rotateToFrameAngle, false, relativeChange);
        while ((!stop && !loopCloserHappened) || (stop && lowBattery)) {
            if (lowBattery) {
                switchBattery();
                stop = false;
                lowBattery = false;
            } else {
                if (!droneRotate && !isBlocked) {
                    if (!localized) {
                        manageDroneCommand("back 20", 3, 2);
                        this->navigateDroneHomePath.push("forward 20");
                        if (!localized) {
                            doTriangulation();
                        }
                    } else {
                        if (!droneNotFly && !isDroneRotate) {
                            if (gettingCloser) {
                                manageDroneCommand("forward 30", 3, 2);
                                this->navigateDroneHomePath.push("back 30");
                            }
                            else{
                                manageDroneCommand("forward 50", 3, 2);
                                this->navigateDroneHomePath.push("back 50");
                            }
                            if (howToRotateToFrame.second)
                                // Pay attention : relativeChange is  a global variable which initialized once
                                relativeChange -= howToRotateToFrame.first;
                            else
                                relativeChange += howToRotateToFrame.first;
                            std::cout << "relative change : " << relativeChange << std::endl;
                            howToRotateToFrame = maintainAngleToPoint(destination, rotateToFrameAngle, false,
                                                                      relativeChange, isHome);
                        }
                        if (areWeNavigatingHome) {
                            std::cout << "going home" << std::endl;
                            if (findAndGoHome(90, true)) {
                                break;
                            }
                            usleep(1500000);
                            continue;
                        }
                    }
                }
            }

            sleep(2);
        }
        std::cout << "size of stack : " << this->navigateDroneHomePath.size() << std::endl;
    }
    else {
        // Navigating back home
        while (!this->navigateDroneHomePath.empty()) {
            if (lowBattery) {
                switchBattery();
                stop = false;
                lowBattery = false;
            } else {
                if (!droneRotate && !isBlocked) {
                    if (!localized) {
                        manageDroneCommand("back 20", 3, 2);
                        this->navigateDroneHomePath.push("forward 20");
                        if (!localized) {
                            doTriangulation();
                        }
                    } else {
                        if (!droneNotFly && !isDroneRotate) {
                            std::string command_drone = this->navigateDroneHomePath.top();
                            this->navigateDroneHomePath.pop();
                            manageDroneCommand(command_drone);
                            std::cout << "size of stack : " << this->navigateDroneHomePath.size() << std::endl;
                        }
                    }
                }
            }
        }
        relativeChange = lastRelativeChange;
    }

    /*if (!droneNotFly) {
        drone->SendCommand("rc 0 0 0 0");
    }*/
    stop = true;
    if (current_drone_mode == noBattery) {
        lowBattery = true;
    }
    current_drone_mode = scanning;
    std::cout << "waiting for the threads to finish" << std::endl;
    monitorDroneProgressThread.join();
    // maintainAngleToPointThread.join();
    //collisionDetectorThread.join();
    std::cout << "threads finished" << std::endl;
    navigationDestination = Point(1000, 1000, 1000);
    orbSlamPointer->GetMapDrawer()->ClearDestinationPoint();
    orbSlamPointer->GetFrameDrawer()->ClearDestinationPoint();
    return Auxiliary::calculateDistanceXY(currentLocation, destination) < closeThreshold * 1.5;
}


void AutonomousDrone::moveWithKeyboard() {
    std::cout << "enter char : " << std::endl;
    char c = getchar();
    switch (c) {
        case 'w':
            manageDroneCommand("forward 30", 3);
            break;
        case 's':
            manageDroneCommand("back 30", 3);
            break;
        case '1':
            manageDroneCommand("up 30", 3);
            break;
        case '2':
            manageDroneCommand("down 30", 3);
            break;
        case 'a':
            manageAngleDroneCommand(25, false, 3);
            break;
        case 'd':
            manageAngleDroneCommand(25, true, 3);
            break;
    }
}


bool AutonomousDrone::manuallyNavigateDrone(const Point &destination, bool rotateToFrameAngle) {
    current_drone_mode = navigation;
    std::cout << "start navigation to:" << destination.to_string() << std::endl;
    navigationDestination = destination;
    orbSlamPointer->GetMapDrawer()->SetDestination(navigationDestination);
    stop = false;
    loopCloserHappened = false;
    weInAWrongScale = false;
    if (Auxiliary::calculateDistanceXY(destination, currentLocation) < closeThreshold) {
        std::cout << "we are close to the point" << std::endl;
        return true;
    }
    //std::thread collisionDetectorThread(&AutonomousDrone::collisionDetector, this, destination);
    std::thread monitorDroneProgressThread(&AutonomousDrone::monitorDroneProgress, this, destination,
                                           false);
    bool areWeNavigatingHome = destination == home;
    while ((!stop && !loopCloserHappened) || (stop && lowBattery)) {
        if (lowBattery) {
            holdMonitorThread = true;
            switchBattery();
            holdMonitorThread = false;
            stop = false;
            lowBattery = false;
        } else {
            moveWithKeyboard();
        }
    }
    stop = true;
    if (current_drone_mode == noBattery) {
        lowBattery = true;
    }
    current_drone_mode = scanning;
    std::cout << "waiting for the threads to finish" << std::endl;
    monitorDroneProgressThread.join();
    //collisionDetectorThread.join();
    std::cout << "threads finished" << std::endl;
    navigationDestination = Point(1000, 1000, 1000);
    orbSlamPointer->GetMapDrawer()->ClearDestinationPoint();
    orbSlamPointer->GetFrameDrawer()->ClearDestinationPoint();
    return Auxiliary::calculateDistanceXY(currentLocation, destination) < closeThreshold * 1.5;
}


void AutonomousDrone::flyToNavigationPoints() {
    Navigation navigation;
    orbSlamPointer->GetMapDrawer()->SetPolygonEdges(currentRoom.exitPoints);
    orbSlamPointer->GetMapDrawer()->DrawMapPoints();
    std::cout << currentRoom.exitPoints.size() << std::endl;
    bool isHome = false;
    for (const Point &point: currentRoom.exitPoints) {
        int battery = drone->GetBatteryStatus();
        if (battery < 40) {
            switchBattery();
            lowBattery = false;
        }
        auto currentMap = getCurrentMap();
        std::pair<Point, Point> track{currentLocation, point};
        auto path = navigation.getNavigationPathByRRT(currentMap, track);
        for (const auto &pathPoint: path) {
            if (navigateDrone(pathPoint, true, isHome)) {
                if (loopCloserHappened) {
                    std::cout << "loop closer happened" << std::endl;
                    break;
                }
                if (weInAWrongScale) {
                    std::cout << "we in wrong scale" << std::endl;
                    break;
                }
                /*if (!checkIfPointInFront(home)) {
                    howToRotate(180, true, true);
                }
                 if (!navigateDrone(home, false) || loopCloserHappened || lowBattery) {
                    break;
                }*/
            }
            if (lowBattery) {
                std::cout << "I'm here" << std::endl;
                switchBattery();
                navigateDrone(pathPoint);
            }
        }
        isHome = !isHome;
    }
    orbSlamPointer->GetMapDrawer()->ClearPolygonEdgesPoint();
    std::cout << "we ended fly to polygon" << std::endl;
    if (saveBinMap) {  // TODO : think if problematic
        char time_buf[21];
        time_t time_tNow;
        std::time(&time_tNow);
        std::strftime(time_buf, 21, "%Y%m%d%H%S%M", gmtime(&time_tNow));
        std::string currentTime(time_buf);
        std::string filename = "/tmp/" + currentTime + ".bin";
        *holdCamera = true;
        sleep(1);
        orbSlamPointer->SaveMap(filename);
        *holdCamera = false;
    }
}


void AutonomousDrone::manuallyFlyToNavigationPoints() {
    Navigation navigation;
    orbSlamPointer->GetMapDrawer()->SetPolygonEdges(currentRoom.exitPoints);
    std::cout << currentRoom.exitPoints.size() << std::endl;
    for (const Point &point: currentRoom.exitPoints) {
        int battery = drone->GetBatteryStatus();
        if (battery < 40) {
            switchBattery();
            lowBattery = false;
        }
        auto currentMap = getCurrentMap();
        std::pair<Point, Point> track{currentLocation, point};
        if (manuallyNavigateDrone(point)) {
            if (loopCloserHappened) {
                std::cout << "loop closer happened" << std::endl;
                break;
            }
        } else {
            std::cout << "we break here" << std::endl;
            break;
        }
        //beginScan(false);

        // if begin scan in comment ("debug" mode) - need to make B.A :  TODO : need to check if work
        /*std::cout << "starting global bundle Adjustments" << std::endl;
        std::thread exhaustiveGlobalAdjustmentThread(&AutonomousDrone::exhaustiveGlobalAdjustment, this);
        exhaustiveGlobalAdjustmentInProgress = true;
        while (exhaustiveGlobalAdjustmentInProgress) {
            doTriangulation();
        }
        exhaustiveGlobalAdjustmentThread.join();*/
    }

    orbSlamPointer->GetMapDrawer()->ClearPolygonEdgesPoint();
    std::cout << "we ended fly to polygon" << std::endl;
    if (saveBinMap) {
        char time_buf[21];
        time_t time_tNow;
        std::time(&time_tNow);
        std::strftime(time_buf, 21, "%Y%m%d%H%S%M", gmtime(&time_tNow));
        std::string currentTime(time_buf);
        std::string filename = "/tmp/" + currentTime + ".bin";
        *holdCamera = true;
        sleep(1);
        orbSlamPointer->SaveMap(filename);
        *holdCamera = false;
    }
}


void AutonomousDrone::stayInTheAir() {
    std::cout << "staying in the air" << std::endl;
    sleep(1);
    while (!lowBattery) {
        if (exitStayInTheAirLoop) {
            break;
        }
        manageDroneCommand("up 30", 3, 5);
        manageDroneCommand("down 30", 3, 5);
    }
}

void AutonomousDrone::connectDrone(bool isReconnect) {
    std::cout << "we trying to reconnect the drone" << std::endl;
    while (true) {
        try {
            std::string commandString = "nmcli c up " + droneWifiName;
            const char *command = commandString.c_str();
            system(command);
            drone.reset(new ctello::Tello());
            statusingDrone = false;
            manageDroneCommand("streamon", 3);
            if (!isReconnect) {
                capture->release();
                sleep(2);
            }
            capture.reset(new cv::VideoCapture(TELLO_STREAM_URL, cv::CAP_FFMPEG));
            sleep(10);
            *holdCamera = false;
            break;
        } catch (...) {
            sleep(2);
        }
    }
}

void AutonomousDrone::exitRoom() {
    beginScan();
    std::thread getNavigationPoints(&AutonomousDrone::getNavigationPoints, this, true);
    stayInTheAir();
    getNavigationPoints.join();
    std::vector<Point> exitPoint{};
    exitPoint.emplace_back(currentRoom.exitPoints[0]);
    currentRoom.exitPoints = exitPoint;
    while (currentRoom.visitedExitPoints.empty()) {
        navigateDrone(currentRoom.exitPoints[0]);
    }
    home = exitPoint[0];
}

void AutonomousDrone::takeOffWithLocalization() {
    *holdCamera = true;
    manageDroneCommand("takeoff", 3, 5);
    *holdCamera = false;
    while (true) {
        if (localized) {
            break;
        }
        *holdCamera = true;
        orbSlamPointer->GetTracker()->Reset();
        *holdCamera = false;

        sleep(5);
        if (!localized) {
            doTriangulation();
        }
    }
    home = currentLocation;
}

void AutonomousDrone::runSimulator(int maxForwardDistance, int forwardAmount) {
    forwardAmount = forwardAmount < 50 ? 50 : forwardAmount;
    std::thread orbThread(&AutonomousDrone::runOrbSlam, this);
    *holdCamera = true;
    while (true) {
        if (canStart) {
            takeOffWithLocalization();
            manageDroneCommand("forward 100", 10, 3);
            buildSimulatorMap(maxRotationAngle);
            while (true) {
                for (int i = 0; i < maxForwardDistance / forwardAmount; ++i) {
                    if (drone->GetBattery() > 50) {
                        manageDroneCommand("forward " + std::to_string(forwardAmount - 30), 10, 5);
                        buildSimulatorMap(maxRotationAngle);
                    } else {
                        std::cout << "no battery going back home to charge" << std::endl;
                        lowBattery = false;
                        auto rotation = getRotationToFrameAngle(home);
                        howToRotate(rotation.first, rotation.second);
                        int amountForReturn = forwardAmount * (i + 1);
                        if (amountForReturn > 500) {
                            int sizeOfLoop = amountForReturn / 500;
                            for (int j = 0; j < sizeOfLoop; ++j) {
                                if (amountForReturn < 500) {
                                    manageDroneCommand("forward " + std::to_string(amountForReturn), 10, 7);
                                    break;
                                } else {
                                    manageDroneCommand("forward " + std::to_string(500), 10, 7);
                                    amountForReturn -= 500;
                                }
                            }
                        } else {
                            manageDroneCommand("forward " + std::to_string(amountForReturn), 10, 7);
                        }

                        /*Charger charger(markers, holdCamera, droneWifiName,
                                        arucoYamlPath, drone, currentImage, dronePort,
                                        false);
                        std::cout << "start running the charger" << std::endl;
                        charger.navigateToBox();*/
                        int timeToSwitchBattery = 30;
                        std::cout << "time to switch battery, you have " +
                                     std::to_string(timeToSwitchBattery) +
                                     " seconds for it" << std::endl;
                        sleep(timeToSwitchBattery);
                        connectDrone();
                        manageDroneCommand("takeoff", 3, 5);
                    }
                }

                std::cout << "new battery" << std::endl;
            }
        } else {
            sleep(2);
        }
    }
}


void AutonomousDrone::switchBattery(int switchingTime) {
    disconnectDrone();
    droneNotFly = true;
    runCamera = false;
    printSomething = true;
    localized = false;
    std::cout << switchingTime << " second for battery switching" << std::endl;
    sleep(switchingTime);
    droneNotFly = false;
    connectDrone(false);
    std::cout << "Drone connected !" << std::endl;
    runCamera = true;
    sleep(2);
    manageDroneCommand("takeoff", 3);
    sleep(1);  // because error "not joystick"
    printSomething = false;
    while (!localized) {
        doTriangulation();
        if (localized) {
            break;
        }
        manageAngleDroneCommand(maxRotationAngle, false, 5, 2);
    }
}


void AutonomousDrone::run() {

    std::thread orbThread(&AutonomousDrone::runOrbSlam, this);
    while (runDrone) {
        if (canStart) {
            std::thread batteryThread(&AutonomousDrone::alertLowBattery, this);
            rooms.emplace_back(Room());
            currentRoom = rooms.back();
            home = Point();
            std::cout << "taking off" << std::endl;
            manageDroneCommand("takeoff", 3);
            sleep(1);  // because error "not joystick"
            manageDroneCommand("up 20", 3); // TODO : delete
            beginScan(false);
            while (runDrone) {
                if (!lowBattery) {
                    std::thread getNavigationPoints(&AutonomousDrone::getNavigationPoints, this, true);
                    stayInTheAir();
                    getNavigationPoints.join();
                    if (stopCondition(currentRoom.exitPoints)) {
                        std::cout << "stop condition held" << std::endl;
                        runDrone = false;
                        break;
                    }
                    if (this->isManual)
                        manuallyFlyToNavigationPoints();
                    else
                        flyToNavigationPoints();
                    /*if (weInAWrongScale && !lowBattery) {
                        std::cout << "going back to base" << std::endl;
                        if (!checkIfPointInFront(home)) {
                            howToRotate(180, true, true);
                        }
                        navigateDrone(home, false);
                    }*/
                }
                if (lowBattery) {
                    std::cout << "no battery" << std::endl;
                    lowBattery = false;
                    if (useCharger) {
                        if (Auxiliary::calculateDistanceXY(Point(), currentLocation) >
                            closeThreshold * 1.2) {
                            if (!checkIfPointInFront(home)) {
                                howToRotate(180, true, true);
                            }
                            navigateDrone(home, false);

                        } else {
                            auto angle = getRotationToFrameAngle(home);
                            howToRotate(angle.first, angle.second);
                            auto distance = distanceToHome();
                            while (distance == 0.0) {
                                howToRotate(maxRotationAngle, angle.second);
                                distance = distanceToHome();
                            }
                        }
                        Charger charger(markers, holdCamera, droneWifiName,
                                        arucoYamlPath, drone, currentImage, dronePort,
                                        false);
                        std::cout << "start running the charger" << std::endl;
                        charger.navigateToBox();
                        connectDrone();
                        manageDroneCommand("takeoff", 3, 5);
                    } else {
                        switchBattery();
                    }
                    if (localized) {
                        howToRotate(180, true, true);
                    } else {
                        //orbSlamPointer->Reset();
                        beginScan();
                    }
                }
                rooms.emplace_back(Room());
                currentRoom = rooms.back();
                std::cout << "new scan" << std::endl;
            }
        } else {
            sleep(2);
        }
    }
    orbSlamRunning = false;
    sleep(10);
}

#pragma clang diagnostic pop
