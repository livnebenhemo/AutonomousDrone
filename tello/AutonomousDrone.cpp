//
// Created by rbdstudent on 15/06/2021.
//

#include "AutonomousDrone.h"

#include <utility>


AutonomousDrone::AutonomousDrone(std::shared_ptr<ctello::Tello> drone,
                                 std::string vocabularyFilePath,
                                 std::string cameraYamlPath,
                                 const std::string &arucoYamlPath,
                                 std::string droneWifiName,
                                 int sizeOfFrameStack,
                                 bool withPlot,
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
    markers.emplace_back(std::pair<int, double>(4, 0.1815));
    markers.emplace_back(std::pair<int, double>(5, 0.1815));
    markers.emplace_back(std::pair<int, double>(6, 0.1815));
    cameraParams = Charger::getCameraCalibration(arucoYamlPath);
    dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_ARUCO_ORIGINAL);
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
    while (runCamera) {
        if (!capture->isOpened() || *holdCamera) {
            sleep(2);
            continue;
        }
        capture->read(*currentImage);
        video.write(*currentImage);
    }
    video.release();
}

void AutonomousDrone::runOrbSlam() {
    std::thread getCameraThread(&AutonomousDrone::getCameraFeed, this);
    ORB_SLAM2::System SLAM(vocFilePath, yamlFilePath, ORB_SLAM2::System::MONOCULAR, true);
    orbSlamPointer = &SLAM;
    orbSlamRunning = true;
    double timeStamp = 0.2;
    int amountOfChanges = 0;
    while (orbSlamRunning) {
        if (!currentImage->empty()) {
            canStart = true;
            cv::Mat orbSlamCurrentPose = orbSlamPointer->TrackMonocular(*currentImage, timeStamp);
            if (orbSlamCurrentPose.empty()) {
                localized = false;
                continue;
            }
            localized = true;
            updateCurrentLocation(orbSlamCurrentPose);
            auto orbSlamTracker = orbSlamPointer->GetTracker();
            if (orbSlamTracker) {
                ORB_SLAM2::Frame orbFrame(orbSlamPointer->GetTracker()->mCurrentFrame);
                auto mvpMapPoints = orbFrame.GetMvpMapPoints();
                if (!mvpMapPoints.empty()) {
                    updateCurrentFrame(orbFrame);
                    int numberOfChanges = orbSlamPointer->GetMap() ? orbSlamPointer->GetMap()->GetLastBigChangeIdx()
                                                                   : 0;
                    if (amountOfChanges != numberOfChanges) {
                        loopCloserHappened = true;
                        amountOfChanges = numberOfChanges;
                    }
                }
            }

        }

        //timeStamp += 0.1;
    }
    orbSlamPointer->Shutdown();
    cvDestroyAllWindows();
}

void AutonomousDrone::updateCurrentLocation(const cv::Mat &Tcw) {
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
    std::vector<double> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    currentLocation = Point(twc.at<float>(0), twc.at<float>(2), twc.at<float>(1), q[0], q[1], q[2], q[3]);
}

bool AutonomousDrone::updateCurrentFrame(ORB_SLAM2::Frame frame) {
    if (frame.getFrameId() != currentFrame.frameId) {
        if (!frame.N) {
            std::cout << "no mvKeys" << std::endl;
            return false;
        }
        auto Rwc = frame.getCameraRotation();
        cv::Mat Tcw = frame.getCameraTranslation();
        if (Rwc.empty()) {
            std::cout << "no Rwc" << std::endl;
            return false;
        }
        int frameId = frame.getFrameId();
        auto q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
        std::vector<Point> framePoints;
        int amountOfKeyPoints = frame.N;
        for (auto p: frame.mvpMapPoints) {
            if (p && !p->isBad()) {
                Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
                framePoints.emplace_back(Point(v.x(), v.z(), v.y(), q[0], q[1], q[2], q[3], frameId));
            }
        }
        auto newFrame = Frame((double) twc.at<float>(0), (double) twc.at<float>(2), (double) twc.at<float>(1),
                              (double) q[0], (double) q[1],
                              (double) q[2], (double) q[3], frameId,
                              framePoints, amountOfKeyPoints);
        currentFrame = newFrame;
        lastFrames.insert(lastFrames.begin(), newFrame);
        if (lastFrames.size() > sizeOfFrameStack) {
            lastFrames.pop_back();
        }
    }
}

std::vector<Point> AutonomousDrone::getCurrentMap() {
    std::vector<Point> currentMap;
    for (auto p: orbSlamPointer->GetMap()->GetAllMapPoints()) {
        if (p && !p->isBad()) {
            auto frame = p->GetReferenceKeyFrame();
            int frameId = frame->mnFrameId;
            cv::Mat Tcw = frame->GetPose();
            auto point = p->GetWorldPos();
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            auto q = ORB_SLAM2::Converter::toQuaternion(Rwc);
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            currentMap.emplace_back(Point(v.x(), v.z(), v.y(), q[0], q[1], q[2], q[3], frameId));
        }
    }
    saveMap(rooms.size());
    return currentMap;
}

void AutonomousDrone::saveMap(int fileNumber) {
    while (true) {
        try {
            auto orbSlamMap = orbSlamPointer->GetMap();
            if (orbSlamMap) {
                std::ofstream pointData;
                pointData.open("/tmp/pointData" + std::to_string(fileNumber) + ".csv");
                for (auto p: orbSlamMap->GetAllMapPoints()) {
                    if (p && !p->isBad()) {
                        auto frame = p->GetReferenceKeyFrame();
                        int frameId = frame->mnFrameId;
                        cv::Mat Tcw = frame->GetPose();
                        auto point = p->GetWorldPos();
                        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
                        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
                        auto q = ORB_SLAM2::Converter::toQuaternion(Rwc);
                        Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
                        pointData << v.x() << "," << v.y() << "," << v.z() << "," << q[0]
                                  << "," << q[1] << "," << q[2] << "," << q[3] << "," << frameId <<
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
    if (!lowBattery) {
        while (commandingDrone || statusingDrone) {
            usleep(200);
        }
        commandingDrone = true;
        while (amountOfAttempt--) {
            if (drone->SendCommandWithResponse(command)) {
                commandingDrone = false;
                if (amountOfSleep) {
                    sleep(amountOfSleep);
                } else {
                    usleep(400000);
                }
                return true;
            }
        }
        sleep(1);
    }
    commandingDrone = false;
    return false;
}

bool AutonomousDrone::doTriangulation() {
    return manageDroneCommand("back 30", 3) && manageDroneCommand("forward 30", 3);
}

void AutonomousDrone::rotateDrone(int angle, bool clockwise, bool buildMap) {
    if (!lowBattery && angle > 5) {
        if (localized) {
            if (clockwise) {
                manageDroneCommand("cw " + std::to_string(angle), 3);
            } else {
                manageDroneCommand("ccw " + std::to_string(angle), 3);
            }
            usleep(300000);
        }
        int amountOfLostLocalizations = 0;
        if (!localized) {
            int regainLocalizationAngle = angle;
            std::cout << "amount of points in frame:" << currentFrame.amountOfKeyPoints << std::endl;
            if (currentFrame.amountOfKeyPoints < 2000) {
                manageDroneCommand("back 30", 1, 1);
            }
            while (!localized && !lowBattery) {
                if (clockwise) {
                    manageDroneCommand("ccw " + std::to_string(regainLocalizationAngle), 3, 1);
                } else {
                    manageDroneCommand("cw " + std::to_string(regainLocalizationAngle), 3, 1);
                }
                regainLocalizationAngle = 25;
                amountOfLostLocalizations += 1;
                if (amountOfLostLocalizations == 3) {
                    manageDroneCommand("back 30", 3, 1);
                }
            }
        }
        if (buildMap || currentFrame.amountOfKeyPoints < 2000) {
            doTriangulation();
        }
        if (amountOfLostLocalizations) {
            amountOfLostLocalizations %= int(360 / 25);
            while (amountOfLostLocalizations - 1) {
                rotateDrone(25, clockwise, true);
                amountOfLostLocalizations--;
            }
            rotateDrone(angle, clockwise, true);
        }
    }
}

void AutonomousDrone::howToRotate(int angle, bool clockwise, bool buildMap) {
    droneRotate = true;
    if (angle < 360) {
        if (angle < 30) {
            rotateDrone(angle, clockwise, buildMap);
        } else {
            int amountOfRotation = ceil(angle / 25);
            while (amountOfRotation--) {
                if (!lowBattery) {
                    rotateDrone(25, clockwise, buildMap);
                }
            }
            rotateDrone(int(angle % 25), clockwise, buildMap);
        }
    }
    droneRotate = false;
}

void AutonomousDrone::disconnectDrone() {
    manageDroneCommand("land", 3, 5);
    manageDroneCommand("streamoff", 3, 3);
}

void AutonomousDrone::alertLowBattery() {
    lowBattery = false;
    int battery;
    int amountOfAttempts = 2;
    while (true) {
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
        if (battery < 50) {
            if (!(amountOfAttempts--)) {
                if (current_drone_mode == navigation && navigationDestination == Point()) {
                    current_drone_mode = noBattery;
                } else {
                    lowBattery = true;
                    stop = true;
                }
                std::cout << "low battery" << std::endl;
                break;
            }
        } else {
            amountOfAttempts = 2;
        }
        sleep(5);
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
            cv::aruco::estimatePoseSingleMarkers(corners, 0.185, cameraParams[0], cameraParams[1],
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
                    return manageDroneCommand("cw " + std::to_string(180 - currentLeftOverAngle), 3, 1);
                else if (currentLeftOverAngle > 10 && currentLeftOverAngle < 40)
                    return manageDroneCommand("ccw " + std::to_string(currentLeftOverAngle), 3, 1);
            } else {
                currentLeftOverAngle =
                        currentLeftOverAngle < -90 ? -1 * (-180 - currentLeftOverAngle) : -1 * currentLeftOverAngle;
                if (currentLeftOverAngle > 10) {
                    if (t.at<double>(0) < 0)
                        return manageDroneCommand("ccw " + std::to_string(currentLeftOverAngle), 3, 1);
                    else
                        return manageDroneCommand("cw " + std::to_string(currentLeftOverAngle), 3, 1);
                }

            }
            return t.at<double>(2);
        }
    }
    return 0;
}

bool AutonomousDrone::findAndGoHome(int howClose, bool stopNavigation) {
    double distance = distanceToHome();
    if (distance) {
        std::cout << "distance home:" << distance << std::endl;
        stop = stopNavigation;
        int forward = int((distance - 1.78) * howClose);
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

void AutonomousDrone::beginScan(bool findHome, int rotationAngle) {
    current_drone_mode = scanning;
    //manageDroneCommand("down 30", 1, 1);

    if (!localized) {
        doTriangulation();
    }
    for (int i = 0; i < ceil(360 / rotationAngle) + 2; i++) {
        if (lowBattery) {
            return;
        }
        if (findHome) {
            if (findAndGoHome(100, false)) {
                home = currentLocation;
                orbSlamPointer->GetMapDrawer()->SetCharger(home);
                findHome = false;
            }
        }
        howToRotate(25, true, true);
        i -= localized ? 0 : 1;
    }
    auto prevLocation = currentLocation;
    manageDroneCommand("up 40", 3, 3);
    isMinusUp = prevLocation.z > currentLocation.z;
    manageDroneCommand("down 40", 3);
}

Point AutonomousDrone::convertFrameToPoint(const Frame &frame) {
    return {frame.x, frame.y, frame.z, frame.qx, frame.qy, frame.qz, frame.qw, frame.frameId};
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
                    if (Auxiliary::calculateDistance(point, visitedExitPoint) <
                        Auxiliary::calculateDistance(currentLocation, point) / 2.5) {
                        exitPoints.erase(exitPoints.begin() + pointIndex);
                    }
                }

        }
        pointIndex += 1;
    }
    currentRoom.exitPoints = exitPoints;
    exitStayInTheAirLoop = true;
    std::cout << "we got the polygon navigation points:" << exitPoints.size() << std::endl;
}

std::pair<int, bool> AutonomousDrone::getRotationToFrameAngle(const Point &point) {
    double desiredYaw = atan2(2.0 * (point.qx * point.qw + point.qy * point.qz),
                              1 - 2 * (pow(point.qy, 2) + pow(point.qz, 2)));
    Point currentPosition = currentLocation;
    double currentYaw = atan2(2.0 * (currentPosition.qx * currentPosition.qw + currentPosition.qy * currentPosition.qz),
                              1 - 2 * (pow(currentPosition.qy, 2) + pow(currentPosition.qz, 2)));
    double angle = Auxiliary::radiansToAngle(desiredYaw) - Auxiliary::radiansToAngle(currentYaw);
    std::cout << "raw quat angle:" << angle << std::endl;
    bool clockwise;
    if (angle < 0) {
        if (angle >= -180) {
            angle *= -1;
            clockwise = isMinusUp;
        } else {
            angle += 360;
            clockwise = !isMinusUp;
        }
    } else {
        if (angle >= 180) {
            clockwise = !isMinusUp;
            angle = 360 - angle;
        } else {
            clockwise = isMinusUp;
        }
    }
    return std::pair<int, bool>{int(angle), clockwise};
}

std::pair<Point, Point> AutonomousDrone::getNavigationVector(const Point &previousPosition, const Point &destination) {
    Point currentPosition = currentLocation;
    Point droneVector = Point(previousPosition.x - currentPosition.x, previousPosition.y - currentPosition.y, 0);
    Point exitVector = Point(destination.x - currentPosition.x, destination.y - currentPosition.y, 0);
    return std::pair<Point, Point>{droneVector, exitVector};
}

void AutonomousDrone::maintainAngleToPoint(const Point &destination, bool rotateToFrameAngle) {
    if (rotateToFrameAngle) {
        auto howToRotateToFrame = getRotationToFrameAngle(destination);
        howToRotate(howToRotateToFrame.first, howToRotateToFrame.second);
    }
    sleep(3);
    while (!stop && !lowBattery) {
        try {
            if (!isBlocked && !droneRotate) {
                Point dronePreviousPosition = currentLocation;
                //sleep(gettingCloser ? 2 : 5);
                if (gettingFurther) {
                    isTurning = true;
                    std::cout << "turning around" << std::endl;
                    howToRotate(180, true, true);
                    gettingFurther = false;
                    sleep(3);
                    isTurning = false;
                    continue;
                }
                sleep(4);
                if (!gettingFurther && !stop && !droneRotate && !isBlocked) {
                    auto navigationVectors = getNavigationVector(dronePreviousPosition, destination);
                    auto rotationDirections = Auxiliary::getRotationToTargetInFront(navigationVectors.first,
                                                                                    navigationVectors.second);
                    if (rotationDirections.first > 8) {
                        howToRotate(rotationDirections.first, rotationDirections.second);
                        sleep(2);
                    }
                }
            } else {
                usleep(500000);
            }
        } catch (std::exception &e) {
            std::cout << e.what() << " in maintainAngleToPoint" << std::endl;
        } catch (...) {
            std::cout << "unknown exception in maintainAngleToPoint" << std::endl;
        }

    }
}

Point AutonomousDrone::protectiveSphereByClosePoint(Point dronePosition, std::vector<Point> points, double sphereRadius,
                                                    double epsilon,
                                                    int minSamples) {
    std::sort(points.begin(), points.end(), [&dronePosition](const Point &p1, const Point &p2) {
        return Auxiliary::calculateDistance(p1, dronePosition) < Auxiliary::calculateDistance(p2, dronePosition);
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

void AutonomousDrone::areWeInWrongScale(const std::vector<Frame> &frames) {
    double sumOfDistances = 0.0;
    for (const auto &frame: frames) {
        sumOfDistances += Auxiliary::calculateDistance(convertFrameToPoint(frame), Point());
    }
    double mean = sumOfDistances / frames.size();
    double variance = 0.0;
    for (const auto &frame: frames) {
        variance += pow(Auxiliary::calculateDistance(convertFrameToPoint(frame), Point()) - mean, 2);
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
                std::cout << "we are in worng scale" << std::endl;
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
                    auto[angle, clockwise, isUp] = checkMotion(lastKnownFrameAsPoint,
                                                               currentLocation, /*frame.points[pointIndex]*/closePoint);
                    if (angle > 0) {
                        isBlocked = true;

                        speed = 0;
                        drone->SendCommand("rc 0 0 0 0");
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

void AutonomousDrone::monitorDroneProgress(const Point &destination) {
    double previousDistance = 10000;
    usleep(500000);
    int i = 0;
    int amountOfGettingFurther = 4;
    closeThreshold = Auxiliary::calculateDistance(currentLocation, destination) / 2.5;
    std::cout << "close threshold:" << closeThreshold << std::endl;
    double expectedScale = closeThreshold * 0.005;
    std::cout << "scale threshold:" << expectedScale << std::endl;
    std::vector<double> distances;
    while (!stop && !lowBattery) {
        try {
            double distance = Auxiliary::calculateDistance(currentLocation, destination);
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
                if (distances.size() > 20) {
                    auto mean = Auxiliary::calculateMeanOfDistanceDifferences(distances);
                    if (!(i % 5)) {
                        std::cout << "mean of distances: " << mean << std::endl;
                    }
                    safetyThreshold = mean * 5;//gettingCloser ? mean * 3 : mean * 2;
                    weInAWrongScale = mean < (gettingCloser ? expectedScale / 2 : expectedScale);
                    distances.erase(distances.begin());
                }
                distances.emplace_back(distance);
                if (distance <= closeThreshold * 1.2) {
                    gettingCloser = true;
                    goUpOrDown(destination);
                    speed = 15;
                }
                if (!isTurning && (distance - previousDistance) > 0.01 && !isBlocked && !droneRotate) {
                    speed = 20;
                    std::cout << "getting further" << std::endl;
                    distances = std::vector<double>{};
                    gettingFurther = --amountOfGettingFurther < 0;
                    orbSlamPointer->GetFrameDrawer()->ClearDestinationPoint();
                } else {
                    amountOfGettingFurther = 4;
                    orbSlamPointer->GetFrameDrawer()->SetDestination(navigationDestination);
                    //speed = distance > 1 ? 30 : 20;
                    gettingFurther = false;
                    speed = distance > closeThreshold * 1.8 ? 25 : 15;
                }
            } else {
                distances = std::vector<double>{};
            }
            previousDistance = distance;
            usleep(450000);
            i++;
        } catch (std::exception &e) {
            std::cout << e.what() << " in monitorDroneProgress" << std::endl;
        } catch (...) {
        }
    }
}

bool AutonomousDrone::navigateDrone(const Point &destination, bool rotateToFrameAngle) {
    current_drone_mode = navigation;
    std::cout << "start navigation to:" << destination.to_string() << std::endl;
    navigationDestination = destination;
    orbSlamPointer->GetMapDrawer()->SetDestination(navigationDestination);
    stop = false;
    loopCloserHappened = false;
    weInAWrongScale = false;
    if (Auxiliary::calculateDistance(destination, currentLocation) < closeThreshold) {
        std::cout << "we are close to the point" << std::endl;
        return true;
    }
    std::thread collisionDetectorThread(&AutonomousDrone::collisionDetector, this, destination);
    std::thread maintainAngleToPointThread(&AutonomousDrone::maintainAngleToPoint, this, destination,
                                           rotateToFrameAngle);
    std::thread monitorDroneProgressThread(&AutonomousDrone::monitorDroneProgress, this, destination);
    bool areWeNavigatingHome = destination == home;
    while (!stop && !loopCloserHappened && !lowBattery) {
        if (!droneRotate && !isBlocked) {
            drone->SendCommand("rc 0 " + std::to_string(speed)
                               + " 0 0");
            if (areWeNavigatingHome) {
                if (findAndGoHome(90, true)) {
                    break;
                }
                usleep(1500000);
                continue;
            }
        }
        sleep(2);
    }
    drone->SendCommand("rc 0 0 0 0");
    stop = true;
    if (current_drone_mode == noBattery) {
        lowBattery = true;
    }
    current_drone_mode = scanning;
    std::cout << "waiting for the threads to finish" << std::endl;
    monitorDroneProgressThread.join();
    maintainAngleToPointThread.join();
    collisionDetectorThread.join();
    std::cout << "threads finished" << std::endl;
    navigationDestination = Point(1000, 1000, 1000);
    orbSlamPointer->GetMapDrawer()->ClearDestinationPoint();
    orbSlamPointer->GetFrameDrawer()->ClearDestinationPoint();
    return Auxiliary::calculateDistance(currentLocation, destination) < closeThreshold * 1.5;
}

void AutonomousDrone::flyToNavigationPoints() {
    orbSlamPointer->GetMapDrawer()->SetPolygonEdges(currentRoom.exitPoints);
    std::cout << currentRoom.exitPoints.size() << std::endl;
    for (const Point &point: currentRoom.exitPoints) {
        if (navigateDrone(point) && !loopCloserHappened && !lowBattery) {
            if (!checkIfPointInFront(home)) {
                howToRotate(180, true, true);
            }
            if (loopCloserHappened || lowBattery || weInAWrongScale) {
                break;
            }
            if (!navigateDrone(home, false) || loopCloserHappened || lowBattery) {
                break;
            }
        } else {
            break;
        }
    }
    orbSlamPointer->GetMapDrawer()->ClearPolygonEdgesPoint();
    std::cout << "we ended fly to polygon" << std::endl;
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

void AutonomousDrone::connectDrone() {
    std::cout << "we trying to reconnect the drone" << std::endl;
    while (true) {
        try {
            drone.reset(new ctello::Tello());
            drone->Bind(dronePort--, dronePort - 1000);
            dronePort--;
            manageDroneCommand("streamon");
            capture->release();
            sleep(2);
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

void AutonomousDrone::run() {

    std::thread orbThread(&AutonomousDrone::runOrbSlam, this);
    while (true) {
        if (canStart) {
            std::thread batteryThread(&AutonomousDrone::alertLowBattery, this);
            rooms.emplace_back(Room());
            currentRoom = rooms.back();
            home = Point();
            manageDroneCommand("takeoff", 3, 3);
            /*exitRoom();*/
            beginScan(true);
            while (true) {
                if (!lowBattery) {
                    std::thread getNavigationPoints(&AutonomousDrone::getNavigationPoints, this, false);
                    stayInTheAir();
                    getNavigationPoints.join();
                    flyToNavigationPoints();
                    if (weInAWrongScale && !lowBattery) {
                        std::cout << "going back to base" << std::endl;
                        if (!checkIfPointInFront(home)) {
                            howToRotate(180, true, true);
                        }
                        navigateDrone(home, false);
                        orbSlamPointer->Reset();
                        beginScan(true);
                    }
                }
                if (lowBattery) {
                    std::cout << "no battery" << std::endl;
                    batteryThread.join();
                    lowBattery = false;
                    if (Auxiliary::calculateDistance(Point(), currentLocation) > closeThreshold * 1.2) {
                        if (!checkIfPointInFront(home)) {
                            howToRotate(180, true, true);
                        }
                        navigateDrone(home, false);

                    }
                    Charger charger(markers, holdCamera, droneWifiName,
                                    arucoYamlPath, drone, currentImage, dronePort,
                                    false);
                    std::cout << "start running the charger" << std::endl;
                    charger.chargeByPaper();
                    connectDrone();
                    batteryThread = std::thread(&AutonomousDrone::alertLowBattery, this);
                    manageDroneCommand("takeoff", 3, 5);
                    if (localized) {
                        howToRotate(180, true, true);
                    } else {
                        orbSlamPointer->Reset();
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