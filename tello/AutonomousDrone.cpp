//
// Created by rbdstudent on 15/06/2021.
//

#include "AutonomousDrone.h"


AutonomousDrone::AutonomousDrone(std::shared_ptr<ctello::Tello> drone,
                                 std::string vocabularyFilePath,
                                 std::string cameraYamlPath,
                                 std::string arucoYamlPath,
                                 std::string droneWifiName,
                                 int sizeOfFrameStack,
                                 bool withPlot,
                                 std::string chargerBluetoothAddress) {
    currentFrame = Frame();
    home = Point();
    this->drone = drone;
    this->vocFilePath = vocabularyFilePath;
    this->yamlFilePath = cameraYamlPath;
    this->withPlot = withPlot;
    this->sizeOfFrameStack = sizeOfFrameStack;
    rooms = std::vector<Room>{};
    this->orbSlamPointer = nullptr;
    exitStayInTheAirLoop = false;
    //this->capture = capture;
    holdCamera = make_shared<bool>(false);
    lastFrames = std::vector<Frame>{};
    currentImage = make_shared<cv::Mat>();
    this->capture = make_shared<cv::VideoCapture>(TELLO_STREAM_URL, cv::CAP_FFMPEG);
    this->chargerBluetoothAddress = chargerBluetoothAddress;
    this->arucoYamlPath = arucoYamlPath;
    this->droneWifiName = droneWifiName;
}

void AutonomousDrone::getCameraFeed() {
    runCamera = true;
    while (runCamera) {
        if (!capture->isOpened() || *holdCamera) {
            sleep(2);
            continue;
        }
        capture->read(*currentImage);
    }
}

void AutonomousDrone::runOrbSlam() {
    std::thread getCameraThread(&AutonomousDrone::getCameraFeed, this);
    ORB_SLAM2::System SLAM(vocFilePath, yamlFilePath, ORB_SLAM2::System::MONOCULAR, true);
    orbSlamPointer = &SLAM;
    orbSlamRunning = true;
    double timeStamp = 0.2;
    int amountOfChanges = 0;
    while (orbSlamRunning) {
        try {
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
        } catch (...) {
            std::cout << "error in runOrbSlam" << std::endl;
        }

        //timeStamp += 0.1;
    }
    orbSlamPointer->Shutdown();
    cvDestroyAllWindows();
}

void AutonomousDrone::updateCurrentLocation(cv::Mat Tcw) {
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
    std::vector<double> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    currentLocation = Point(twc.at<float>(0), twc.at<float>(2), twc.at<float>(1), q[0], q[1], q[2], q[3]);
}

bool AutonomousDrone::updateCurrentFrame(ORB_SLAM2::Frame frame) {
    if (frame.getFrameId() != currentFrame.frameId) {
        if (frame.mvKeys.empty()) {
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
        int amountOfKeyPoints = frame.mvKeys.size();
        for (auto p : frame.mvpMapPoints) {
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
    for (auto p : orbSlamPointer->GetMap()->GetAllMapPoints()) {
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
                for (auto p : orbSlamMap->GetAllMapPoints()) {
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

bool AutonomousDrone::manageDroneCommand(std::string command, int amountOfAttempt, int amountOfSleep) {
    if (!lowBattery) {
        commandingDrone = true;
        while (amountOfAttempt--) {
            while (true) {
                if (!statusingDrone) {
                    if (drone->SendCommandWithResponse(command)) {
                        commandingDrone = false;
                        if (amountOfSleep) {
                            sleep(amountOfSleep);
                        }
                        return true;
                    } else {
                        break;
                    }
                } else {
                    usleep(200);
                }
            }
            sleep(1);
        }
        commandingDrone = false;
    }
    return false;
}

bool AutonomousDrone::doTriangulation() {
    return manageDroneCommand("forward 30", 3) && manageDroneCommand("back 30", 3);
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
            if (currentFrame.amountOfKeyPoints < 1000) {
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
        if (buildMap || currentFrame.amountOfKeyPoints < 1000) {
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

void AutonomousDrone::beginScan(bool findHome, int rotationAngle) {
    current_drone_mode = scanning;
    //manageDroneCommand("down 30", 1, 1);
    const std::vector<cv::Mat> cameraParams = Charger::getCameraCalibration(arucoYamlPath);
    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_ARUCO_ORIGINAL);
    if (!localized) {
        doTriangulation();
    }
    for (int i = 0; i < ceil(360 / rotationAngle) + 2; i++) {
        if (lowBattery) {
            return;
        }
        if (findHome) {
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            cv::aruco::detectMarkers(*currentImage, dictionary, corners, ids);
            if (!ids.empty()){
                for (auto id : ids){
                    std::cout << "we found:" << id << std::endl;
                }
                std::vector<cv::Vec3d> localRvecs, localTvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, 0.185, cameraParams[0], cameraParams[1],
                                                     localRvecs,
                                                     localTvecs);
                cv::Mat rmat = cv::Mat::eye(3, 3, CV_64FC1);
                try {
                    cv::Rodrigues(localRvecs[0], rmat);
                } catch (...) {
                    std::cout << "coudlnot convert vector to mat" << std::endl;
                    continue;
                }
                auto t = cv::Mat(-rmat.t() * cv::Mat(localTvecs[0]));
                manageDroneCommand("forward "+std::to_string(t.at<double>(2)*80),3,2);
                home = currentLocation;
            }
        }
        howToRotate(25, true, true);
        usleep(500000);
        i -= localized ? 0 : 1;
    }
    isMinusUp = true;
}

Point AutonomousDrone::convertFrameToPoint(Frame frame) {
    return Point(frame.x, frame.y, frame.z, frame.qx, frame.qy, frame.qz, frame.qw, frame.frameId);
}

void AutonomousDrone::getNavigationPoints(bool isExit) {
    std::cout << "getting navigation points" << std::endl;
    exitStayInTheAirLoop = false;
    currentRoom.points = std::vector<Point>{};
    currentRoom.points = getCurrentMap();
    std::cout << "we saved points" << std::endl;
    Polygon polygon(currentRoom.points, isExit);
    std::vector<Point> exitPoints = polygon.getExitPointsByPolygon(withPlot);
    int pointIndex = 0;
    for (auto point : exitPoints) {
        for (auto room : rooms) {
            for (auto visitedExitPoint : room.visitedExitPoints) {
                if (Auxiliary::calculateDistance(point, visitedExitPoint) <
                    Auxiliary::calculateDistance(currentLocation, point) / 4.0) {
                    exitPoints.erase(exitPoints.begin() + pointIndex);
                }
            }

        }
        pointIndex += 1;
    }
    currentRoom.exitPoints = exitPoints;
    exitStayInTheAirLoop = true;
    std::cout << "we got the polygon navigation points:" <<exitPoints.size() << std::endl;
}

std::pair<int, bool> AutonomousDrone::getRotationToFrameAngle(Point point) {
    double desiredYaw = atan2(2.0 * (point.qx * point.qw + point.qy * point.qz),
                              1 - 2 * (pow(point.qy, 2) + pow(point.qz, 2)));
    Point currentPosition = currentLocation;
    double currentYaw = atan2(2.0 * (currentPosition.qx * currentPosition.qw + currentPosition.qy * currentPosition.qz),
                              1 - 2 * (pow(currentPosition.qy, 2) + pow(currentPosition.qz, 2)));
    double angle = Auxiliary::radiansToAngle(desiredYaw) - Auxiliary::radiansToAngle(currentYaw);
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

std::pair<Point, Point> AutonomousDrone::getNavigationVector(Point previousPosition, Point destination) {
    Point currentPosition = currentLocation;
    Point droneVector = Point(previousPosition.x - currentPosition.x, previousPosition.y - currentPosition.y, 0);
    Point exitVector = Point(destination.x - currentPosition.x, destination.y - currentPosition.y, 0);
    return std::pair<Point, Point>{droneVector, exitVector};
}

void AutonomousDrone::maintainAngleToPoint(Point destination, bool rotateToFrameAngle) {
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

int
AutonomousDrone::protectiveSphere(Point dronePosition, std::vector<Point> points, double sphereRadius, double epsilon,
                                  int minSamples) {
    int pointIndex = 0;
    for (Point point : points) {
        pointIndex += 1;
        if (Auxiliary::calculateDistance3D(point, dronePosition) < sphereRadius) {
            int counter = 0;
            for (Point point2 : points) {
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
AutonomousDrone::checkMotion(Point oldestPosition, Point currentPosition, Point closePoint, int angleRange) {
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

void AutonomousDrone::areWeInWrongScale(std::vector<Frame> frames) {
    double sumOfDistances = 0.0;
    for (auto frame : frames) {
        sumOfDistances += Auxiliary::calculateDistance(convertFrameToPoint(frame), Point());
    }
    double mean = sumOfDistances / frames.size();
    double variance = 0.0;
    for (auto frame : frames) {
        variance += pow(Auxiliary::calculateDistance(convertFrameToPoint(frame), Point()) - mean, 2);
    }
    weInAWrongScale = variance < 0.001;
}

void AutonomousDrone::collisionDetector(Point destination) {
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
                int pointIndex = protectiveSphere(convertFrameToPoint(frame), frame.points, safetyThreshold);
                if (pointIndex != -1) {
                    std::cout << "we are blocked" << std::endl;
                    Point lastKnownFrameAsPoint = convertFrameToPoint(lastKnownFrame);
                    auto[angle, clockwise, isUp] = checkMotion(lastKnownFrameAsPoint,
                                                               convertFrameToPoint(frame), frame.points[pointIndex]);
                    if (angle > 0) {
                        isBlocked = true;
                        speed = 0;
                        drone->SendCommand("rc 0 0 0 0");
                        usleep(300000);
                        auto navigationVectors = getNavigationVector(lastKnownFrameAsPoint, destination);
                        auto rotationDirections = Auxiliary::getRotationToTargetInFront(navigationVectors.first,
                                                                                        navigationVectors.second);
                        if (isUp == 1) {
                            manageDroneCommand("up 20", 3);
                        } else if (isUp == -1) {
                            manageDroneCommand("down 20", 3);
                        }
                        if (rotationDirections.second == clockwise) {
                            if (rotationDirections.first > angle) {
                                std::cout << "can avoid with navigate to destintion" << std::endl;
                                howToRotate(rotationDirections.first, rotationDirections.second);
                            } else {

                                if (angle > 60) {
                                    std::cout << "same direction and blocked from sides" << std::endl;
                                    if (clockwise) {
                                        manageDroneCommand("left 20", 3);
                                    } else {
                                        manageDroneCommand("right 20", 3);
                                    }
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
                }
            } else {
                if (currentFrame.amountOfKeyPoints < 1000) {
                    manageDroneCommand("back 60", 3, 1);
                    if (isMinusUp) {
                        if (destination.z - closeThreshold > currentFrame.z) {
                            drone->SendCommand("rc 0 0 -10 0");
                            usleep(1500000);
                        } else if (destination.z + closeThreshold < currentFrame.z) {
                            drone->SendCommand("rc 0 0 -10 0");
                            usleep(1500000);
                        }
                        drone->SendCommand("0 0 0 0");
                    } else {
                        if (destination.z + closeThreshold < currentFrame.z) {
                            drone->SendCommand("rc 0 0 -15 0");
                            sleep(2);
                        } else if (destination.z - closeThreshold > currentFrame.z) {
                            drone->SendCommand("rc 0 0 -15 0");
                            sleep(2);
                        }
                    }
                } else {
                    manageDroneCommand("back 30", 3, 1);
                    manageDroneCommand("forward 30", 3, 1);
                }
            }
        }
        isBlocked = false;
        usleep(300000);
    }
}

void AutonomousDrone::monitorDroneProgress(Point destination) {
    double previousDistance = 10000;
    usleep(500000);
    int i = 0;
    int amountOfGettingFurther = 4;
    closeThreshold = Auxiliary::calculateDistance(currentLocation, destination) / 4.0;
    std::cout << "close threshold:" << closeThreshold << std::endl;
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
            if (!droneRotate) {
                if (distances.size() > 20) {
                    auto mean = Auxiliary::calculateMeanOfDistanceDifferences(distances);
                    if (!(i % 5)) {
                        std::cout << "mean of distances: " << mean << std::endl;
                    }
                    safetyThreshold = mean * 3;//gettingCloser ? mean * 3 : mean * 2;
                    weInAWrongScale = mean < (gettingCloser ? 0.0007 / 2 : 0.0007);
                    distances.erase(distances.begin());
                }
                distances.emplace_back(distance);
                gettingCloser = distance <= closeThreshold * 1.5;
                if (!isTurning && (distance - previousDistance) > 0.01 && !isBlocked && !droneRotate) {
                    speed = 20;
                    std::cout << "getting further" << std::endl;
                    gettingFurther = --amountOfGettingFurther < 0;
                    orbSlamPointer->GetFrameDrawer()->ClearDestinationPoint();
                } else {
                    amountOfGettingFurther = 4;
                    orbSlamPointer->GetFrameDrawer()->SetDestination(navigationDestination);
                    //speed = distance > 1 ? 30 : 20;
                    gettingFurther = false;
                }
            }
            previousDistance = distance;
            usleep(500000);
            i++;
        } catch (std::exception e) {
            std::cout << e.what() << " in monitorDroneProgress" << std::endl;
        } catch (...) {
        }
    }
}

bool AutonomousDrone::navigateDrone(Point destination, bool rotateToFrameAngle) {
    current_drone_mode = navigation;
    std::cout << "start navigation to:" << destination.to_string() << std::endl;
    navigationDestination = destination;
    orbSlamPointer->GetMapDrawer()->SetDestination(navigationDestination);
    stop = false;
    loopCloserHappened = false;
    weInAWrongScale = false;
    double distanceToPoint = Auxiliary::calculateDistance(currentLocation, destination);
    std::cout << "distance to point:" << distanceToPoint << std::endl;
    if (distanceToPoint < closeThreshold) {
        return true;
    }
    std::thread collisionDetectorThread(&AutonomousDrone::collisionDetector, this, destination);
    std::thread maintainAngleToPointThread(&AutonomousDrone::maintainAngleToPoint, this, destination,
                                           rotateToFrameAngle);
    std::thread monitorDroneProgressThread(&AutonomousDrone::monitorDroneProgress, this, destination);
    while (!stop && !loopCloserHappened && !lowBattery) {
        try {
            if (!droneRotate && !isBlocked) {
                drone->SendCommand("rc 0 " + std::to_string(!gettingCloser ? speed : 15)
                                   + " 0 0");
            }
        } catch (std::exception e) {
            std::cout << e.what() << std::endl;
        } catch (...) {

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
    for (Point point: currentRoom.exitPoints) {
        if (navigateDrone(point) && !loopCloserHappened && !lowBattery) {
            if (Auxiliary::calculateDistance(home, currentLocation) > closeThreshold) {
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
            drone->Bind(dronePort--);
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

void AutonomousDrone::run() {
    std::vector<std::pair<int, double>> markers;
    markers.emplace_back(std::pair<int, double>(4, 0.1815));
    markers.emplace_back(std::pair<int, double>(5, 0.1815));
    std::thread orbThread(&AutonomousDrone::runOrbSlam, this);
    while (true) {
        if (canStart) {
            std::thread batteryThread(&AutonomousDrone::alertLowBattery, this);
            rooms.emplace_back(Room());
            currentRoom = rooms.back();
            manageDroneCommand("takeoff", 3, 3);
            /*beginScan();
            std::thread getNavigationPoints(&AutonomousDrone::getNavigationPoints, this, true);
            stayInTheAir();
            getNavigationPoints.join();
            std::vector<Point> exitPoint{};
            exitPoint.emplace_back(currentRoom.exitPoints[0]);
            currentRoom.exitPoints = exitPoint;
            while (currentRoom.visitedExitPoints.empty()){
                navigateDrone(currentRoom.exitPoints[0]);
            }
            home = exitPoint[0];*/
            beginScan(true);
            while (true) {
                if (!lowBattery) {
                    std::thread getNavigationPoints(&AutonomousDrone::getNavigationPoints, this, false);
                    stayInTheAir();
                    getNavigationPoints.join();
                    flyToNavigationPoints();
                    if (weInAWrongScale && !lowBattery) {
                        orbSlamPointer->Reset();
                        beginScan();
                    }
                }
                if (lowBattery) {
                    std::cout << "no battery" << std::endl;
                    batteryThread.join();
                    lowBattery = false;
                    if (Auxiliary::calculateDistance(Point(), currentLocation) > closeThreshold * 1.2) {
                        howToRotate(180, true);
                        navigateDrone(home, false);

                    }
                    Charger charger(markers, chargerBluetoothAddress, capture, holdCamera, droneWifiName,
                                    arucoYamlPath, drone, currentImage,
                                    false);
                    std::cout << "start running the charger" << std::endl;
                    //charger.run();
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