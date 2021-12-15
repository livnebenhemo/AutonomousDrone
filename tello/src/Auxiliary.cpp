//
// Created by rbdstudent on 17/06/2021.
//

#include "tello/include/Auxiliary.h"

Point Auxiliary::GetCenterOfMass(const std::vector<Point> &points) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    for (const Point &point : points) {
        x += point.x;
        y += point.y;
        z += point.z;
    }
    int size = points.size();
    return {x / size, y / size, z / size, 0, 0, 0, 0};
}

double Auxiliary::det(const Point& point1, const Point& point2) {
    return point1.x * point2.y - point1.y * point2.x;
}

double Auxiliary::angleToRadians(int angle) {
    return (angle * M_PI) / 180;
}

double Auxiliary::radiansToAngle(double radian) {
    return radian * (180 / M_PI);
}

std::vector<double> Auxiliary::getXValues(const std::vector<Point>& points) {
    std::vector<double> xValues;
    for (const Point& point : points) {
        xValues.push_back(point.x);
    }
    return xValues;
}

std::vector<double> Auxiliary::getYValues(const std::vector<Point>& points) {
    std::vector<double> yValues;
    for (const Point& point : points) {
        yValues.push_back(point.y);
    }
    return yValues;
}

//#ifdef NOTRPI
void Auxiliary::showCloudPoint(const std::vector<Point>& redPoints, const std::vector<Point>& cloud) {
    matplotlibcpp::clf();
    matplotlibcpp::scatter(getXValues(cloud), getYValues(cloud), 2.0);
    matplotlibcpp::plot(getXValues(redPoints), getYValues(redPoints), "ro");
    matplotlibcpp::show();
}

//#endif
double Auxiliary::distanceBetweenPointAndSegment(const Point& point, Line segment) {
    auto point1 = segment.getPoint1();
    auto point2 = segment.getPoint2();
    Point segmentDifference(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
    double dot = (point.x - point1.x) * segmentDifference.x + (point.y - point1.y) * segmentDifference.y;
    double segmentLength = pow(segmentDifference.x, 2) + pow(segmentDifference.y, 2);
    double param = -1;
    Point distancePoint(0, 0, 0);
    if (segmentLength != 0) {
        param = dot / segmentLength;
    }
    if (param < 0) {
        distancePoint.x = point1.x;
        distancePoint.y = point1.y;
    } else if (param > 1) {
        distancePoint.x = point2.x;
        distancePoint.y = point2.y;
    } else {
        distancePoint.x = point1.x + param * segmentDifference.x;
        distancePoint.y = point1.y + param * segmentDifference.y;
    }
    return calculateDistance(Point(point.x - distancePoint.x, point.y - distancePoint.y, 0), Point(0, 0, 0));
}

double Auxiliary::getDistanceToClosestSegment(const Point& point, const std::vector<Line>& segments) {
    double minDistance = 10000;
    for (const auto& segment : segments) {
        double distance = distanceBetweenPointAndSegment(point, segment);
        minDistance = minDistance > distance ? distance : minDistance;
    }
    return minDistance;
}

double Auxiliary::getAngleFromSlope(double slope) {
    return radiansToAngle(atan(slope));
}
double Auxiliary::calculateDistance3D(const Point& point1, const Point& point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2) + pow(point2.z - point1.z,2));
}
double Auxiliary::calculateDistance(const Point& point1, const Point& point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
}

std::pair<int, bool>
Auxiliary::getRotationToTargetInFront(const Point& previous, const Point& current, const Point& destination, bool isMinusUp) {
    double PreviousToCurrent = atan2(previous.y - current.y, previous.x - current.x);
    double CurrentToDistance = atan2(destination.y - current.y, destination.x - current.x);
    double angle = Auxiliary::radiansToAngle(CurrentToDistance-PreviousToCurrent);
    std::cout << "angle:" << angle << std::endl;
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
    if (angle > 90){
        angle = 180 - angle;
    }
    return std::pair<int, bool>{angle, clockwise};
}

std::pair<int, bool> Auxiliary::getRotationToTargetInFront(const Point& point1, const Point& point2) {
    Eigen::Vector3d vector1(point1.x, point1.y,0);
    Eigen::Vector3d vector2(point2.x, point2.y,0);
    Eigen::Vector3d unitVector1 = vector1 / vector1.norm();
    Eigen::Vector3d unitVector2 = vector2 / vector2.norm();
    int angle = int(radiansToAngle(acos(unitVector1.dot(unitVector2))));
    bool clockwise = unitVector1.cross(unitVector2).z() <= 0;
    if (angle > 90) {
        angle = 180 - angle;
        clockwise = !clockwise;
    }
    return std::pair<int, bool>{angle, clockwise};
}

long Auxiliary::myGcd(long a, long b)
{
    if (a == 0)
        return b;
    else if (b == 0)
        return a;

    if (a < b)
        return myGcd(a, b % a);
    else
        return myGcd(b, a % b);
}
std::tuple<int,int,int> Auxiliary::getRationalInverse(double input)
{
    double integral = std::floor(input);
    double frac = input - integral;

    const long precision = 1000000000; // This is the accuracy.

    long myGcd_ = myGcd(std::round(frac * precision), precision);

    long denominator = precision / myGcd_;
    long numerator = round(frac * precision) / myGcd_;

    std::cout << integral << " + ";
    std::cout << numerator << " / " << denominator << std::endl;
    return std::tuple(integral,numerator,denominator);
}
double Auxiliary::calculateMeanOfDistanceDifferences(std::vector<double> distances){
    double sumOfDistances = 0.0;
    for (int i = 0; i <distances.size()-1; ++i) {
        sumOfDistances +=std::abs(distances[i] - distances[i+1]);
    }
    return sumOfDistances/(distances.size()-1);
}
double Auxiliary::calculateVariance(const std::vector<double>& distances){
    double sumOfDistances = 0.0;
    for (auto distance : distances)
    {
        sumOfDistances+= distance;
    }
    double mean = sumOfDistances/distances.size();
    double variance = 0.0;
    for (auto distance : distances){
        variance+=pow(distance - mean,2);
    }
    return variance;
}
double Auxiliary::getAngleBySlopes(Line line1, Line line2) {
    Eigen::Vector3d vector1(1, line1.getSlope(), 0);
    Eigen::Vector3d vector2(1, line2.getSlope(), 0);
    Eigen::Vector3d unitVector1 = vector1 / vector1.norm();
    Eigen::Vector3d unitVector2 = vector2 / vector2.norm();
    return radiansToAngle(acos(unitVector1.dot(unitVector2)));
}