//
// Created by rbdstudent on 26/05/2021.
//

#include "tello/include/Charger.h"

#include <utility>
#include <opencv2/imgproc.hpp>

Charger::Charger(std::vector<std::pair<int, double>> markers, std::shared_ptr<bool> holdCamera,
                 std::string droneWifiName,
                 std::string telloYamlFilePath, std::shared_ptr<ctello::Tello> drone, std::shared_ptr<cv::Mat> frame,
                 int currentPort, bool withImShow,
                 std::string chargerBluetoothAddress,
                 int raspberryToTelloPinNumber,
                 double slowSpeedDistance,
                 double fastSpeedDistance,
                 double distanceFromWall,
                 double distanceToWall,
                 double distanceUpDownMarker,
                 double distanceRightFromArucoCenter,
                 double distanceLeftFromArucoCenter, double almostStopSpeedDistance,
                 std::vector<double> speedScaleFactor, int sizeOfMovingAverageWindow, int distanceToBox) {
    this->distanceLeftFromArucoCenter = distanceLeftFromArucoCenter;
    this->distanceRightFromArucoCenter = distanceRightFromArucoCenter;
    this->distanceToWall = distanceToWall;
    this->distanceFromWall = distanceFromWall;
    this->fastSpeedDistance = fastSpeedDistance;
    this->slowSpeedDistance = slowSpeedDistance;
    this->distanceUpDownMarker = distanceUpDownMarker;
    this->markers = std::move(markers);
    this->holdCamera = std::move(holdCamera);
    this->chargerBluetoothAddress = std::move(chargerBluetoothAddress);
    //this->capture = capture;
    this->frame = std::move(frame);
    this->currentPort = currentPort--;
    this->telloYamlFilePath = std::move(telloYamlFilePath);
    this->drone = std::move(drone);
    this->raspberryToTelloPinNumber = raspberryToTelloPinNumber;
    this->withImShow = withImShow;
    this->droneWifiName = std::move(droneWifiName);
    this->speedScaleFactor = std::move(speedScaleFactor);
    this->sizeOfMovingAverageWindow = sizeOfMovingAverageWindow;
    this->distanceToBox = distanceToBox;
    widthForLanding = 360;
    heightForLanding = 240;
    //wiringPiSetup();
    //pinMode(raspberryToTelloPinNumber,OUTPUT);
}

void Charger::resetTrackingGlobals() {
    leftOverAngle = {-1, false};
}

std::tuple<float, float, cv::Point2f> Charger::getArucoInfo(std::vector<cv::Point2f> corners) {

    double leftNorm = cv::norm(cv::Point2f(corners[0].x - corners[3].x, corners[0].y - corners[3].y));
    double rightNorm = cv::norm(cv::Point2f(corners[1].x - corners[2].x, corners[1].y - corners[2].y));
    double upNorm = cv::norm(cv::Point2f(corners[1].x - corners[0].x, corners[1].y - corners[0].y));
    double downNorm = cv::norm(cv::Point2f(corners[2].x - corners[3].x, corners[2].y - corners[3].y));
    return {leftNorm / rightNorm, (upNorm + downNorm) / 2, (corners[0] + corners[1] + corners[2] + corners[3]) / 4};
}

void Charger::handleQueue(std::vector<double> &queue, double newValue) const {
    if (queue.size() == sizeOfMovingAverageWindow) {
        queue.pop_back();
    }
    queue.insert(queue.begin(), newValue);
}

std::tuple<int, int, int, int> Charger::landInBox(std::tuple<float, float, cv::Point2f> info,
                                                  std::vector<double> &yawErrorQueue,
                                                  std::vector<double> &upDownQueue, std::vector<double> &leftRightQueue,
                                                  std::vector<double> &forwardBackwardQueue, double errorYawEpsilon,
                                                  double errorLeftRightEpsilon, double errorForwardBackwardEpsilon,
                                                  double errorUpDownEpsilon, double factor = 1.0) {
    auto height = drone->GetHeightState();
    std::cout << height << std::endl;
    if (height < 50) {//TODO: check if this height sampling is correct
        double lCharger = 60.0;
        double aAruco = 30.0;
        double errorFactor = 5.0;
        double speedFactor = 2.0;
        double leftRightError = leftRightQueue.front();
        double forwardBackwardError = forwardBackwardQueue.front();
        double upDownError = upDownQueue.front();
        double yawError = yawErrorQueue.front();
        if (std::abs(upDownError) < errorFactor * errorUpDownEpsilon &&
            std::abs(yawError) < errorFactor * errorYawEpsilon &&
            std::abs(leftRightError) < errorFactor * errorLeftRightEpsilon
            && std::abs(forwardBackwardError) < errorFactor * errorForwardBackwardEpsilon) {
            speedFactor = 0.8;
        }
        double currentYawError = 70 * (1 - std::get<0>(info));
        currentYawError = std::accumulate(yawErrorQueue.begin(), yawErrorQueue.end() - 1, currentYawError) /
                          sizeOfMovingAverageWindow;
        std::cout << "current yaw error:" << currentYawError << std::endl;
        int yawSpeed = std::floor((std::clamp(
                speedScaleFactor[0] * currentYawError * 12 + speedScaleFactor[1] * (currentYawError - yawError) * 2,
                -70.0, 70.0)));
        double currentLeftRightError = (std::get<2>(info).x - std::round(widthForLanding / 2)) / 3;
        currentLeftRightError =
                std::accumulate(leftRightQueue.begin(), leftRightQueue.end() - 1, currentLeftRightError) /
                sizeOfMovingAverageWindow;
        std::cout << "current left right error: " << currentLeftRightError << std::endl;

        int leftRightSpeed = std::floor(std::clamp(2 * speedScaleFactor[0] * currentLeftRightError +
                                                   speedScaleFactor[1] * (currentLeftRightError - leftRightError),
                                                   -30.0, 30.0) * speedFactor);
        double theta = std::atan((drone->GetHeightState() - aAruco) / lCharger);
        double dCharger = lCharger / std::cos(theta);
        double thetaNom = std::atan((30 - aAruco) / lCharger);
        double dChargerNom = lCharger / std::cos(thetaNom);
        double dRatio = dChargerNom / dCharger;
        double currentUpDownError = (drone->GetHeightState() - 30) * 2;
        currentUpDownError = (std::accumulate(upDownQueue.begin(), upDownQueue.end() - 1, 0.0) + upDownError) /
                             sizeOfMovingAverageWindow;
        std::cout << "current up down error: " << currentUpDownError << std::endl;

        int upDownSpeed = -12;//(-4 * speedScaleFactor[0]*currentUpDownError - speedScaleFactor[1]*(currentUpDownError - upDownError))*0.5
        double currentForwardBackwardError = (std::get<1>(info) / dRatio - 48);
        currentForwardBackwardError =
                std::accumulate(forwardBackwardQueue.begin(), forwardBackwardQueue.end() - 1,
                                currentForwardBackwardError) /
                sizeOfMovingAverageWindow;
        std::cout << "current forward backward error: " << currentForwardBackwardError << std::endl;

        int forwardBackwardSpeed = std::floor(std::clamp(-3 * (speedScaleFactor[0] * currentForwardBackwardError +
                                                               speedScaleFactor[1] *
                                                               (currentForwardBackwardError - forwardBackwardError) *
                                                               2), -18 * speedFactor, 18 * speedFactor));
        handleQueue(leftRightQueue, currentLeftRightError);
        handleQueue(yawErrorQueue, currentYawError);
        handleQueue(forwardBackwardQueue, currentForwardBackwardError);
        handleQueue(upDownQueue, currentUpDownError);
        return {leftRightSpeed, forwardBackwardSpeed, upDownSpeed, yawSpeed};
    }
    return {0, 0, 0, 0};

}

void Charger::navigateToBox() {
    manageDroneCommand("down 30", 3, 1);
    std::vector<double> yawErrorQueue{10}, upDownQueue{10}, leftRightQueue{10}, forwardBackwardQueue{10};
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    int amountOfTimesInTheBox = 0;
    double factor = 1.0;
    double errorYawEpsilon = 1.1 * factor;
    double errorLeftRightEpsilon = 8 * factor;
    double errorForwardBackwardEpsilon = 8 * factor;
    double errorUpDownEpsilon = 10 * factor;
    double errorSideEpsilon = 8.0;
    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_6X6_250);
    while (!stop) {
        bool canContinue = false;
        cv::Mat currentFrame;
        cv::resize(*frame, currentFrame, cv::Size(360, 240));
        if (currentFrame.empty())
            continue;
        cv::aruco::detectMarkers(currentFrame, dictionary, corners, ids);
        int rightId = 0;
        while (rightId < ids.size()) {
            if (ids[rightId] != 0) {
                canContinue = true;
                break;
            }
            rightId++;
        }
        if (canContinue) {
            auto info = getArucoInfo(corners[rightId]);
            auto rcSpeeds = landInBox(info, yawErrorQueue, upDownQueue, leftRightQueue, forwardBackwardQueue,
                                      errorYawEpsilon, errorLeftRightEpsilon, errorForwardBackwardEpsilon,
                                      errorUpDownEpsilon);
            double errorSide = leftRightQueue.front() -
                               (60 + forwardBackwardQueue.front()) * std::sin(yawErrorQueue.front() * M_PI / 180);
            std::cout << "error side: " << errorSide << std::endl;
            if (std::abs(upDownQueue.front()) < errorUpDownEpsilon && std::abs(errorSide) < errorSideEpsilon &&
                std::abs(yawErrorQueue.front()) < errorYawEpsilon &&
                std::abs(forwardBackwardQueue.front()) < errorForwardBackwardEpsilon) {
                amountOfTimesInTheBox += 1;
                std::cout << "in a box: " << amountOfTimesInTheBox << std::endl;
                if (amountOfTimesInTheBox == 20) {
                    drone->SendCommand("rc 0 0 0 0");
                    usleep(250);
                    double forwardBackwardFactor = 1.0;
                    if (std::abs(forwardBackwardQueue.front()) >= 5) {
                        forwardBackwardFactor = 1.2;
                    }
                    int lastStep = 30;
                    manageDroneCommand("forward " + std::to_string(
                            int(lastStep - forwardBackwardQueue.front() * forwardBackwardFactor)), 3, 1);
                    manageDroneCommand("land");
                    return;
                }
            } else {
                amountOfTimesInTheBox = 0;
            }
            if (std::get<2>(info).x != 0) {
                std::cout
                        << "rc " + std::to_string(std::get<0>(rcSpeeds)) + " " + std::to_string(std::get<1>(rcSpeeds)) +
                           " " + std::to_string(std::get<2>(rcSpeeds)) + " " + std::to_string(std::get<3>(rcSpeeds))
                        << std::endl;
                drone->SendCommand(
                        "rc " + std::to_string(std::get<0>(rcSpeeds)) + " " + std::to_string(std::get<1>(rcSpeeds)) +
                        " " + std::to_string(std::get<2>(rcSpeeds)) + " " + std::to_string(std::get<3>(rcSpeeds)));
                usleep(250);
            } else {
                drone->SendCommand("rc 0 0 0 0");
            }

            cv::putText(currentFrame, "lr: " + std::to_string(leftRightQueue.front()), cv::Point2i(20, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
            cv::putText(currentFrame, "fb: " + std::to_string(forwardBackwardQueue.front()), cv::Point2i(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
            cv::putText(currentFrame, "yaw: " + std::to_string(yawErrorQueue.front()), cv::Point2i(20, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
            cv::imshow("debug info", currentFrame);
        }
    }

}

void Charger::trackMarker() {
    stop = false;
    std::cout << "started track thread" << std::endl;
    std::vector<std::vector<cv::Point2f>> corners;
    const std::vector<cv::Mat> cameraParams = getCameraCalibration(telloYamlFilePath);
    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_ARUCO_ORIGINAL);
    while (!stop) {
        std::vector<int> ids;
        if (!frame->empty()) {
            cv::aruco::detectMarkers(*frame, dictionary, corners, ids);
        } else {
            std::cout << "no frames" << std::endl;
            resetTrackingGlobals();
            continue;
        }
        bool canContinue = false;
        int rightId = 0;
        while (rightId < ids.size()) {
            if (ids[rightId] == currentMarker) {
                canContinue = true;
                break;
            }
            rightId++;
        }
        // if at least one marker detected
        if (canContinue) {
            std::vector<cv::Vec3d> localRvecs, localTvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, currentMarkerSize, cameraParams[0], cameraParams[1],
                                                 localRvecs,
                                                 localTvecs);
            if (!localRvecs.empty()) {
                cv::Mat rmat = cv::Mat::eye(3, 3, CV_64FC1);
                try {
                    cv::Rodrigues(localRvecs[rightId], rmat);
                } catch (...) {
                    std::cout << "coudlnot convert vector to mat" << std::endl;
                    continue;
                }
                auto t = cv::Mat(-rmat.t() * cv::Mat(localTvecs[rightId]));
                rightLeft = t.at<double>(0);
                upDown = t.at<double>(1);
                forward = t.at<double>(2);
                leftOverAngle = getLeftOverAngleFromRotationVector(localRvecs[rightId]);
                usleep(amountOfUSleepForTrackMarker);
            } else {
                resetTrackingGlobals();
            }
        } else {
            resetTrackingGlobals();
        }
        // usleep(100000);
    }
}

void Charger::getCameraFeed() {
    while (!stopCameraThread) {
        try {
            if (!*holdCamera) {
                capture->read(*frame);
                if (!frame->empty()) {
                    cameraOpen = true;
                    if (withImShow) {
                        imshow("CTello Stream", *frame);
                        if (cv::waitKey(1) == 27) {
                            break;
                        }
                    }
                }
            } else {
                sleep(1);
            }
        } catch (...) {
            break;
        }
    }
}

std::vector<cv::Mat> Charger::getCameraCalibration(const std::string &path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("CameraParameters::readFromXMLFile could not open file:" + path);
    int w = -1, h = -1;
    cv::Mat MCamera, MDist;

    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["distortion_coefficients"] >> MDist;
    fs["camera_matrix"] >> MCamera;

    if (MCamera.cols == 0 || MCamera.rows == 0) {
        fs["Camera_Matrix"] >> MCamera;
        if (MCamera.cols == 0 || MCamera.rows == 0)
            throw cv::Exception(9007, "File :" + path + " does not contains valid camera matrix",
                                "CameraParameters::readFromXML", __FILE__, __LINE__);
    }

    if (w == -1 || h == 0) {
        fs["image_Width"] >> w;
        fs["image_Height"] >> h;
        if (w == -1 || h == 0)
            throw cv::Exception(9007, "File :" + path + " does not contains valid camera dimensions",
                                "CameraParameters::readFromXML", __FILE__, __LINE__);
    }
    if (MCamera.type() != CV_32FC1)
        MCamera.convertTo(MCamera, CV_32FC1);

    if (MDist.total() < 4) {
        fs["Distortion_Coefficients"] >> MDist;
        if (MDist.total() < 4)
            throw cv::Exception(9007, "File :" + path + " does not contains valid distortion_coefficients",
                                "CameraParameters::readFromXML", __FILE__, __LINE__);
    }
    MDist.convertTo(MDist, CV_32FC1);
    std::vector<cv::Mat> newCameraParams = {MCamera, MDist};
    return newCameraParams;
}

int Charger::sendMsg(int socket, const std::string &msg, int amountOfAttempts = 10) {
    int status;

    do {
        status = send(socket, msg.c_str(), msg.length(), 0);
        fprintf(stdout, "Status = %d\n", status);
        sleep(2);
    } while (status <= 0 && amountOfAttempts--);
    return amountOfAttempts > 0;
}

void Charger::getEulerAngles(cv::Mat &rotCameraMatrix, cv::Vec3d &eulerAngles) {
    cv::Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
    auto *_r = rotCameraMatrix.ptr<double>();
    double projMatrix[12] = {_r[0], _r[1], _r[2], 0,
                             _r[3], _r[4], _r[5], 0,
                             _r[6], _r[7], _r[8], 0};
    decomposeProjectionMatrix(cv::Mat(3, 4, CV_64FC1, projMatrix),
                              cameraMatrix,
                              rotMatrix,
                              transVect,
                              rotMatrixX,
                              rotMatrixY,
                              rotMatrixZ,
                              eulerAngles);
}

bool Charger::manageDroneCommand(const std::string &command, int amountOfAttempt, int amountOfSleep) {
    while (amountOfAttempt--) {
        if (drone->SendCommandWithResponse(command)) {
            if (amountOfSleep) {
                sleep(amountOfSleep);
            }
            return true;
        }
        sleep(1);
    }
    return false;
}

std::string Charger::readMsg(int socket) {
    char buf[1024] = {0};
    recv(socket, buf, sizeof(buf), 0);
    fprintf(stdout, "%s", buf);
    return {buf};
}

bool Charger::closeCharger(int socket, bool closeSocket) {
    int amountOfAttempts = 5;
    bool isClosed = false;
    sendMsg(socket, "close");
    std::string response;
    sleep(2);
    while (amountOfAttempts--) {
        response = readMsg(socket);
        std::cout << response << std::endl;
        if (response.find("ERROR charger not close") == std::string::npos) {
            break;

        }
        sendMsg(socket, "close");
    }
    sendMsg(socket, "start_fans");
    std::cout << "servo closed" << std::endl;
    if (closeSocket) {
        close(socket);
    }
    return isClosed;
}

bool Charger::openCharger(int socket, bool closeSocket) {
    std::cout << "send command to open charger" << std::endl;
    if (socket == -1) {
        return false;
    }
    int amountOfAttempts = 5;
    bool isOpened = false;
    if (sendMsg(socket, "open") == 4) {
        return true;
    }
    sleep(2);
    std::string response;
    while (amountOfAttempts--) {
        response = readMsg(socket);
        std::cout << response << std::endl;
        if (response.find("ERROR charger not open") == std::string::npos) {
            break;
        }
        sendMsg(socket, "open");
    }
    if (closeSocket) {
        close(socket);
    }
    return isOpened;
}

int Charger::getChargerSocket(std::string connectionAddr) {
    int s;
    struct sockaddr_rc addr = {0};
    int status;
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = 1;
    str2ba(connectionAddr.c_str(), &addr.rc_bdaddr);
    status = connect(s, (struct sockaddr *) &addr, sizeof(addr));
    std::cout << "connection status:" << status << std::endl;
    // std::cout << "socket id:" << s << std::endl;
    if (status) {
        close(s);
        return -1;
    }
    return s;
}

std::string Charger::getMovementInDepth(double forwardBackwards) {
    std::string forwardText;
    if (forwardBackwards > distanceFromWall) {
        forwardText = getForwardSpeedText(forwardBackwards);
    } else if (forwardBackwards < distanceToWall) {
        forwardText = getBackwardsSpeedText(forwardBackwards);
    } else {
        forwardText = "0";
    }
    return forwardText;
}

bool Charger::chargeByEstimation(int batteryAtStart) {
    int battery = 0;
    while (battery == 0) {
        battery = drone->GetBatteryStatus();
    }
    std::cout << "battery before charge:" << battery << std::endl;
    while (true) {
        turnDroneOnOrOff(5);
        std::cout << "drone is off" << std::endl;
        std::cout << "sleeping for:" << 48 * (batteryAtStart - battery) << std::endl;
        sleep(48 * (100 - battery));
        turnDroneOnOrOff();
        connectToDrone(droneWifiName);
        drone.reset(new ctello::Tello());
        drone->Bind(currentPort, currentPort - 1000);
        currentPort--;
        battery = drone->GetBatteryStatus();
        std::cout << "battery after charge:" << battery << std::endl;
        if (100 - battery <= 10) {
            break;
        }
    }
}

bool Charger::communicateWithCharger(int socket, bool closeSocket) {
    closeCharger(socket, false);
    int amountOfLowCurrent = 3;
    if (socket != -1) {
        readMsg(socket);
        auto response = std::string();
        while (true) {
            response = readMsg(socket);
            if (response.find("current") != std::string::npos && response.find("open_close") == std::string::npos) {
                int current;
                try {
                    int commandIndex = response.find(':');
                    current = std::stoi(response.substr(commandIndex + 1, response.length() - 1));
                } catch (...) {
                    continue;
                }
                if (current <= 200) {
                    amountOfLowCurrent--;
                    if (!amountOfLowCurrent) {
                        openCharger(socket);
                        sleep(5);
                        close(socket);
                        return true;
                    }

                } else {
                    amountOfLowCurrent = 5;
                }
            } else if (response.find("open_close") != std::string::npos) {
                break;
            }
            sleep(20);
        }
    }
    if (closeSocket) {
        close(socket);
    }
    return true;
}

bool Charger::correctDroneAngle(std::pair<int, bool> currentLeftOverAngle) {
    int angle = currentLeftOverAngle.first;
    angle = angle < 0 ? angle < -90 ? -1 * (-180 - angle) : -1 * angle : angle > 90 ? 180 - angle : angle;
    bool clockwise = currentLeftOverAngle.second;
    if (angle > 15) {
        if (clockwise)
            return manageDroneCommand("cw " + std::to_string(angle), 3, 1);
        else
            return manageDroneCommand("ccw " + std::to_string(angle), 3, 1);
    }
    return false;
}

std::string Charger::getForwardSpeedText(double distance) const {
    if (distance < distanceFromWall + almostStopSpeedDistance) {
        return "4";
    } else if (distance < distanceFromWall + slowSpeedDistance) {
        return "7";
    } else if (distance > distanceFromWall + fastSpeedDistance) {
        return "15";
    }
    return "11";
}

std::string Charger::getBackwardsSpeedText(double distance) const {
    if (distance > distanceToWall - almostStopSpeedDistance) {
        return "-4";
    } else if (distance > distanceToWall - slowSpeedDistance) {
        return "-7";
    } else if (distance < distanceFromWall - fastSpeedDistance) {
        return "-20";
    }
    return "-12";
}

std::string Charger::getLeftSpeedText(double distance) const {
    if (distance < distanceLeftFromArucoCenter + almostStopSpeedDistance) {
        return "-4";
    }
    if (distance < distanceLeftFromArucoCenter + slowSpeedDistance) {
        return "-6";
    } else if (distance > distanceLeftFromArucoCenter + fastSpeedDistance) {
        return "-12";
    }
    return "-10";
}

std::string Charger::getRightSpeedText(double distance) const {
    if (distance > distanceRightFromArucoCenter - almostStopSpeedDistance) {
        return "4";
    }
    if (distance > distanceRightFromArucoCenter - slowSpeedDistance) {
        return "6";
    } else if (distance < distanceRightFromArucoCenter - fastSpeedDistance) {
        return "12";
    }
    return "10";
}

void Charger::monitorDroneState() {
    while (!stop) {
        auto speed = drone->GetSpeedStatus();
        auto acc = drone->GetAccelerationStatus();
        auto height = drone->GetHeightStatus();
        std::cout << "speed:" << speed << std::endl;
        std::cout << "acc:" << acc << std::endl;
        std::cout << "height:" << height << std::endl;
        sleep(1);
    }
}

void Charger::calculateAxisErrors(double rightLeftExpectation, double forwardBackwardsExpectation,
                                  double upDownExpectation) {
    int amount = amountOfMeasurement;
    double rightLeftSum = 0;
    double forwardBackwardsSum = 0;
    double upDownSum = 0;
    while (amount--) {
        rightLeftSum += rightLeft;
        forwardBackwardsSum += forward;
        upDownSum += upDown;
        usleep(amountOfUSleepForTrackMarker);
    }
    leftRightError = rightLeftSum / rightLeftExpectation;
    forwardBackwardError = forwardBackwardsSum / forwardBackwardsExpectation;
    upDownError = upDownSum / upDownExpectation;
}

double Charger::navigateToMarkerByExpectation(float markerSize, int markerId) {
    std::cout << "starting navigation to:" << markerId << std::endl;
    currentMarkerSize = markerSize;
    currentMarker = markerId;
    sleep(1);
    int amountOfSearchTurns = 1;
    std::string forwardText = "0";
    std::string leftRightText = "0";
    std::string upDownText = "0";
    drone->SendCommand("rc 0 0 0 0");
    double localDistance = 0.0;
    std::pair<int, bool> localLeftOverAngle = {-1, false};
    bool goUp = true;
    double leftRightExpectation = (distanceRightFromArucoCenter - rightLeft) / 10;
    double forwardBackwardsExpectation = (distanceToWall + 0.2 - forward) / 10;
    double upDownExpectation = 0;
    int leftRightSpeed = rightLeft < distanceLeftFromArucoCenter ? 10 : -10;
    int forwardBackwardSpeed = forward > distanceFromWall ? 10 : -10;
    int upDownSpeed = 0;
    while (!stop) {
        if (leftOverAngle.first != -1) {
            amountOfSearchTurns = 1;
            std::cout << "distance:" << forward << std::endl;
            if (localDistance < distanceFromWall && localDistance > distanceToWall
                && rightLeft < distanceRightFromArucoCenter && rightLeft > distanceLeftFromArucoCenter) {
                drone->SendCommand("rc 0 0 0 0");
                break;
            }
            std::cout << "right left:" << rightLeft << std::endl;
            //std::cout << localDistance << std::endl;
            localLeftOverAngle = leftOverAngle;
            correctDroneAngle(leftOverAngle);
            std::string rcCommand(std::to_string(leftRightSpeed) + " " + std::to_string(forwardBackwardSpeed) + " " +
                                  std::to_string(upDownSpeed) + " 0");
            drone->SendCommand(rcCommand);
            std::cout << rcCommand << std::endl;
            std::thread errorCalculationThread(&Charger::calculateAxisErrors, this, leftRightExpectation,
                                               forwardBackwardsExpectation, upDownExpectation);
            usleep(amountOfUSleepForDroneRcCommand);
            errorCalculationThread.join();
            auto[leftRightIntegral, leftRightNumerator, leftRightDenominator] = Auxiliary::getRationalInverse(
                    leftRightError);
            leftRightSpeed *= floor(leftRightDenominator / leftRightNumerator) * (leftRightIntegral > 0 ? -1 : 1);
            auto[forwardBackwardIntegral, forwardBackwardNumerator, forwardBackwardDenominator] = Auxiliary::getRationalInverse(
                    forwardBackwardError);
            forwardBackwardSpeed *= floor(forwardBackwardDenominator / forwardBackwardNumerator) *
                                    (forwardBackwardIntegral > 0 ? -1 : 1);
            auto[upDownIntegral, upDownNumerator, upDownDenominator] = Auxiliary::getRationalInverse(upDownError);
            upDownSpeed *= floor(upDownDenominator / upDownNumerator) * (upDownIntegral > 0 ? -1 : 1);
        } else {
            leftRightText = "0";
            std::string angleText = localLeftOverAngle.second ? "ccw 20" : "cw 20";
            if (upDown <= -0.2) {
                manageDroneCommand("up " + std::to_string(int(upDown * -100)));
            } else if (upDown >= 0.2) {
                manageDroneCommand("down " + std::to_string(int(upDown * 100)));
            }
            upDown = 0;
            if (amountOfSearchTurns++ % 18 == 0) {
                manageDroneCommand("back 30", 2, 1);
                if (goUp) {
                    manageDroneCommand("up 30", 2, 1);
                } else {
                    manageDroneCommand("down 30", 2, 1);
                }
                goUp = !goUp;
            }
            manageDroneCommand(angleText);
        }
    }
    int amountOfShutdowns = 1;
    std::string resistanceLeft = std::to_string(std::stoi(leftRightText) * -1);
    std::string resistanceDown = std::to_string(std::stoi(upDownText) * -1);
    std::string resistanceForward = std::to_string(std::stoi(forwardText) * -1);
    while (amountOfShutdowns--) {
        drone->SendCommand("rc " + resistanceLeft + " " + resistanceForward + " " + resistanceDown
                           + " 0");
        usleep(150000);
    }
    drone->SendCommand("rc 0 0 0 0");

    return localDistance;
}

double Charger::navigateToMarker(float markerSize, int markerId) {
    std::cout << "starting navigation to:" << markerId << std::endl;
    currentMarkerSize = markerSize;
    currentMarker = markerId;
    sleep(1);
    int amountOfSearchTurns = 1;
    std::string forwardText = "0";
    std::string leftRightText = "0";
    std::string upDownText = "0";
    drone->SendCommand("rc 0 0 0 0");
    double localDistance = 0.0;
    std::pair<int, bool> localLeftOverAngle = {-1, false};
    bool goUp = true;
    while (!stop) {
        if (leftOverAngle.first != -1) {
            amountOfSearchTurns = 1;
            localDistance = forward;
            std::cout << "distance:" << localDistance << std::endl;
            if (localDistance < distanceFromWall && localDistance > distanceToWall
                && rightLeft < distanceRightFromArucoCenter && rightLeft > distanceLeftFromArucoCenter) {
                drone->SendCommand("rc 0 0 0 0");
                break;
            } else if (localDistance > distanceFromWall) {
                forwardText = getForwardSpeedText(localDistance);
            } else if (localDistance < distanceToWall) {
                forwardText = getBackwardsSpeedText(localDistance);
            } else {
                forwardText = "0";
            }
            double localRightLeft = rightLeft;
            std::cout << "right left:" << localRightLeft << std::endl;
            if (localRightLeft < distanceLeftFromArucoCenter) {
                leftRightText = getRightSpeedText(localRightLeft);
            } else if (localRightLeft > distanceRightFromArucoCenter) {
                leftRightText = getLeftSpeedText(localRightLeft);
            } else {
                leftRightText = "0";
            }
            //std::cout << localDistance << std::endl;
            localLeftOverAngle = leftOverAngle;
            correctDroneAngle(leftOverAngle);
            if (localDistance > 2) {
                manageDroneCommand("forward " + std::to_string(int((forward - distanceFromWall) * 80)), 3, 1);
            }
            if (upDown > distanceUpDownMarker) {
                upDownText = "-10";
            } else if (upDown < -distanceUpDownMarker) {
                upDownText = "10";
            } else {
                upDownText = "0";
            }
            std::string commandText("rc " + leftRightText + " " + forwardText + " " + upDownText + " 0");
            drone->SendCommand(commandText);
            std::cout << commandText << std::endl;
            usleep(amountOfUSleepForDroneRcCommand);
        } else {
            leftRightText = "0";
            std::string angleText = localLeftOverAngle.second ? "ccw 20" : "cw 20";
            if (upDown <= -0.2) {
                manageDroneCommand("up " + std::to_string(int(upDown * -100)));
            } else if (upDown >= 0.2) {
                manageDroneCommand("down " + std::to_string(int(upDown * 100)));
            }
            upDown = 0;
            if (amountOfSearchTurns++ % 18 == 0) {
                manageDroneCommand("back 30", 2, 1);
                if (goUp) {
                    manageDroneCommand("up 30", 2, 1);
                } else {
                    manageDroneCommand("down 30", 2, 1);
                }
                goUp = !goUp;
            }
            manageDroneCommand(angleText);
        }
    }
    int amountOfShutdowns = 1;
    std::string resistanceLeft = std::to_string(std::stoi(leftRightText) * -1);
    std::string resistanceDown = std::to_string(std::stoi(upDownText) * -1);
    std::string resistanceForward = std::to_string(std::stoi(forwardText) * -1);
    while (amountOfShutdowns--) {
        drone->SendCommand("rc " + resistanceLeft + " " + resistanceForward + " " + resistanceDown
                           + " 0");
        usleep(150000);
    }
    drone->SendCommand("rc 0 0 0 0");

    return localDistance;
}

std::pair<int, bool> Charger::getLeftOverAngleFromRotationVector(const cv::Vec<double, 3> &rvec) {
    cv::Mat R33 = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Rodrigues(rvec, R33);
    cv::Vec3d eulerAngles;
    getEulerAngles(R33, eulerAngles);
    int yaw = eulerAngles[0];
    // std::cout << eulerAngles << std::endl;
    return {yaw, eulerAngles[1] >= 0};
}

bool Charger::connectToDrone(const std::string &droneName) {
    std::string commandString = "nmcli c up " + droneName;
    const char *command = commandString.c_str();
    return system(command) == 0;
}

void Charger::turnDroneOnOrOff(int amountOfSleep) {
    int raspberryPiSocket = getChargerSocket("B8:27:EB:FC:14:FA");
    while (raspberryPiSocket == -1) {
        sleep(1);
        raspberryPiSocket = getChargerSocket("B8:27:EB:FC:14:FA");
    }
    close(raspberryPiSocket);
    sleep(amountOfSleep);
}

void Charger::travelTo3Points() {
    stopCameraThread = false;
    stop = false;
    int batteryAtStart = drone->GetBatteryStatus();
    // manageDroneCommand("up 20");
    //std::thread cameraThead(&Charger::getCameraFeed, this);
    sleep(2);
    std::thread trackerThead(&Charger::trackMarker, this);
    //std::thread monitorDroneStateThead(&Charger::monitorDroneState, this);
    stop = false;
    double chargerDistanceFromWall = distanceFromWall;
    distanceFromWall *= 2;
    distanceToWall = distanceFromWall - 0.6;
    distanceRightFromArucoCenter += 0.4;
    distanceLeftFromArucoCenter -= 0.4;
    manageDroneCommand("takeoff", 3, 3);

    navigateToMarker(markers[0].second, markers[0].first);
    manageDroneCommand("cw 90", 3, 1);
    manageDroneCommand("forward 200", 3, 1);
    navigateToMarker(markers[2].second, markers[2].first);
    manageDroneCommand("cw 180", 3, 1);
    manageDroneCommand("forward 350", 3, 1);
    manageDroneCommand("cw 90", 3, 1);
    distanceFromWall = chargerDistanceFromWall;
    distanceToWall = distanceFromWall - 0.02;
    distanceRightFromArucoCenter -= 0.4;
    distanceLeftFromArucoCenter += 0.4;
    navigateToMarker(markers[0].second, markers[0].first);
    manageDroneCommand("down 30", 3, 1);
    navigateToMarker(markers[1].second, markers[1].first);
    drone->SendCommand("land");
    sleep(5);
    std::cout << "landed!" << std::endl;
    *holdCamera = true;
    stop = true;
    stopCameraThread = true;
    //monitorThead.join();
    trackerThead.join();
    chargeByEstimation(batteryAtStart);
}

void Charger::chargeByPaper() {
    stopCameraThread = false;
    stop = false;
    int batteryAtStart = drone->GetBatteryStatus();
    // manageDroneCommand("up 20");
    //std::thread cameraThead(&Charger::getCameraFeed, this);
    sleep(2);
    std::thread trackerThead(&Charger::trackMarker, this);
    stop = false;
    navigateToMarker(markers[0].second, markers[0].first);
    manageDroneCommand("down 30", 3, 1);
    navigateToMarker(markers[1].second, markers[1].first);
    drone->SendCommand("land");
    sleep(5);
    std::cout << "landed!" << std::endl;
    *holdCamera = true;
    stop = true;
    stopCameraThread = true;
    //monitorThead.join();
    trackerThead.join();
    chargeByEstimation(batteryAtStart);
}


bool Charger::run() {
    int chargerSocket;
    stopCameraThread = false;
    stop = false;
    // manageDroneCommand("up 20");
    //std::thread cameraThead(&Charger::getCameraFeed, this);
    sleep(2);
    std::thread trackerThead(&Charger::trackMarker, this);
    stop = false;
    navigateToMarker(markers[0].second, markers[0].first);
    sleep(1);
    navigateToMarker(markers[1].second, markers[1].first);
    drone->SendCommand("land");

    sleep(5);
    std::cout << "landed!" << std::endl;
    *holdCamera = true;
    stop = true;
    stopCameraThread = true;
    //monitorThead.join();
    trackerThead.join();
    //cameraThead.join();
    drone->closeSockets();
    turnDroneOnOrOff();
    chargerSocket = -1;
    while (chargerSocket == -1) {
        chargerSocket = getChargerSocket(chargerBluetoothAddress);
        sleep(1);
    }
    communicateWithCharger(chargerSocket, true);
    chargerSocket = -1;
    while (chargerSocket == -1) {
        chargerSocket = getChargerSocket(chargerBluetoothAddress);
        sleep(1);
    }
    sendMsg(chargerSocket, "stop_fans");
    close(chargerSocket);
    turnDroneOnOrOff();
    connectToDrone(droneWifiName);
    return true;
}
