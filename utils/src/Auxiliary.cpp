//
// Created by rbdstudent on 17/06/2021.
//

#include <fstream>
#include "include/Auxiliary.h"
#include "opencv2/core.hpp"

Point Auxiliary::rotationMatrixToEulerAngles(cv::Mat &R) {

    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                          R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return {x * 180 / CV_PI, y * 180 / CV_PI, z * 180 / CV_PI};
}

Point Auxiliary::GetCenterOfMass(const std::vector<Point> &points) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    for (const Point &point: points) {
        x += point.x;
        y += point.y;
        z += point.z;
    }
    int size = points.size();
    return {x / size, y / size, z / size};
}

double Auxiliary::det(const Point &point1, const Point &point2) {
    return point1.x * point2.y - point1.y * point2.x;
}

double Auxiliary::angleToRadians(int angle) {
    return (angle * M_PI) / 180;
}

double Auxiliary::radiansToAngle(double radian) {
    return radian * (180 / M_PI);
}

std::vector<double> Auxiliary::getXValues(const std::vector<Point> &points) {
    std::vector<double> xValues;
    for (const Point &point: points) {
        xValues.push_back(point.x);
    }
    return xValues;
}

std::vector<double> Auxiliary::getYValues(const std::vector<Point> &points) {
    std::vector<double> yValues;
    for (const Point &point: points) {
        yValues.push_back(point.y);
    }
    return yValues;
}

std::vector<double> Auxiliary::getZValues(const std::vector<Point> &points) {
    std::vector<double> yValues;
    for (const Point &point: points) {
        yValues.push_back(point.z);
    }
    return yValues;
}

std::tuple<std::vector<cv::Point2d>, std::vector<double>, cv::Point2d>
Auxiliary::pca(const std::vector<Point> &points) {
    int sz = static_cast<int>(points.size());
    cv::Mat data_pts = cv::Mat(sz, 2, CV_64F);
    for (int i = 0; i < data_pts.rows; i++) {
        data_pts.at<double>(i, 0) = points[i].x;
        data_pts.at<double>(i, 1) = points[i].y;
    }
    //Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
    //Store the center of the object
    cv::Point2d cntr = cv::Point2d(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));
    //Store the eigenvalues and eigenvectors
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; i++) {
        eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                    pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
    }

    return {eigen_vecs, eigen_val, cntr};

}

//#ifdef NOTRPI
void Auxiliary::showCloudPoint(const std::vector<Point> &redPoints, const std::vector<Point> &cloud) {
    matplotlibcpp::clf();
    matplotlibcpp::scatter(getXValues(cloud), getYValues(cloud), 2.0);
    matplotlibcpp::plot(getXValues(redPoints), getYValues(redPoints), "ro");
    matplotlibcpp::plot(getXValues(redPoints), getYValues(redPoints));
    matplotlibcpp::show();
}

void Auxiliary::exportToXYZFile(const std::vector<Point> &points, std::string fileName) {
    std::ofstream pointData;
    pointData.open(fileName);
    for (const auto &point: points) {
        pointData << point.x << " " << point.y << " " << point.z << std::endl;
    }
    pointData.close();
}

void Auxiliary::SetupPangolin(const std::string &window_name) {
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(window_name, 640, 480);

    // enable depth
    glEnable(GL_DEPTH_TEST);

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void
Auxiliary::drawPathPangolin(const std::vector<Point> &cloud, std::vector<Point> &path, const std::string &windowName,
                            const std::pair<Point, Point> &lineFromCenter) {
    pangolin::BindToContext(windowName);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY)
    );

    pangolin::Renderable tree;
    tree.Add(std::make_shared<pangolin::Axis>());

    // Create Interactive View in window
    pangolin::SceneHandler handler(tree, s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
            .SetHandler(&handler);

    d_cam.SetDrawFunction([&](pangolin::View &view) {
        view.Activate(s_cam);
        tree.Render();
    });

    while (!pangolin::ShouldQuit()) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPointSize(1);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 1.0);
        for (const auto &point: cloud) {
            glVertex3f(point.x, -1 * point.z, point.y);
        }
        glEnd();
        glPointSize(10);
        glBegin(GL_POINTS);
        glColor3f(1.0, 1.0, 1.0);
        for (const auto &point: path) {
            glVertex3f(point.x, -1 * point.z, point.y);
        }
        glEnd();
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glLineWidth(3);
        for (int i = 0; i < path.size() - 1; ++i) {

            glVertex3f(path[i].x, -1 * path[i].z, path[i].y);
            glVertex3f(path[i + 1].x, -1 * path[i + 1].z, path[i + 1].y);
        }
        glEnd();
        glBegin(GL_LINES);
        glColor3f(1.0, 1.0, 1.0);
        glLineWidth(3);
        glVertex3f(path[path.size() - 2].x, -1 * path[path.size() - 2].z, path[path.size() - 2].y);
        glVertex3f(path.back().x, -1 * path.back().z, path.back().y);
        glEnd();
        glColor3f(1.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(lineFromCenter.first.x, -1 * lineFromCenter.first.z, lineFromCenter.first.y);
        glVertex3f(lineFromCenter.second.x, -1 * lineFromCenter.second.z, lineFromCenter.second.y);
        glEnd();
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void Auxiliary::DrawMapPointsPangolin(const std::vector<Point> &cloud, const std::vector<Point> &redPoints,
                                      const std::string &windowName,
                                      const std::pair<Point, Point> &lineFromCenter) {
    pangolin::BindToContext(windowName);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY)
    );

    pangolin::Renderable tree;
    tree.Add(std::make_shared<pangolin::Axis>());

    // Create Interactive View in window
    pangolin::SceneHandler handler(tree, s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
            .SetHandler(&handler);

    d_cam.SetDrawFunction([&](pangolin::View &view) {
        view.Activate(s_cam);
        tree.Render();
    });

    while (!pangolin::ShouldQuit()) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPointSize(1);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 1.0);

        for (const auto &point: cloud) {
            glVertex3f(point.x, -1 * point.z, point.y);
        }
        glEnd();

        glPointSize(10);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        for (const auto &point: redPoints) {
            glVertex3f(point.x, -1 * point.z, point.y);
        }
        glEnd();
        glLineWidth(3);
        glColor3f(1.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(lineFromCenter.first.x, -1 * lineFromCenter.first.z, lineFromCenter.first.y);
        glVertex3f(lineFromCenter.second.x, -1 * lineFromCenter.second.z, lineFromCenter.second.y);
        glEnd();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void Auxiliary::showGraph(std::vector<double> &x, std::vector<double> &y, const std::string &pointsDisplay) {
    matplotlibcpp::plot(x, y, pointsDisplay);
    matplotlibcpp::show();
}

void Auxiliary::showCloudPoint3D(const std::vector<Point> &redPoints, const std::vector<Point> &cloud) {
    //matplotlibcpp::axis("3d");
    matplotlibcpp::scatter(getXValues(cloud), getYValues(cloud), getZValues(cloud), 0.1);
    matplotlibcpp::show();
}


//#endif
double Auxiliary::distanceBetweenPointAndSegment(const Point &point, Line segment) {
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
    return calculateDistanceXY(Point(point.x - distancePoint.x, point.y - distancePoint.y, 0), Point(0, 0, 0));
}

double Auxiliary::getDistanceToClosestSegment(const Point &point, const std::vector<Line> &segments) {
    double minDistance = 10000;
    for (const auto &segment: segments) {
        double distance = distanceBetweenPointAndSegment(point, segment);
        minDistance = minDistance > distance ? distance : minDistance;
    }
    return minDistance;
}

double Auxiliary::getAngleFromSlope(double slope) {
    return radiansToAngle(atan(slope));
}

double Auxiliary::calculateDistance3D(const Point &point1, const Point &point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2) + pow(point2.z - point1.z, 2));
}

double Auxiliary::calculateDistanceXY(const Point &point1, const Point &point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
}

double Auxiliary::calculateDistanceXZ(const Point &point1, const Point &point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.z - point1.z, 2));
}

std::string Auxiliary::GetDataSetsDirPath() {
    char currentDirPath[256];
    getcwd(currentDirPath, 256);
    std::string settingPath = currentDirPath;
    settingPath += "/../datasets/";
    return settingPath;
}

std::pair<int, bool>
Auxiliary::getRotationToTargetInFront(const Point &previous, const Point &current, const Point &destination,
                                      bool isMinusUp) {
    double PreviousToCurrent = atan2(previous.y - current.y, previous.x - current.x);
    double CurrentToDistance = atan2(destination.y - current.y, destination.x - current.x);
    double angle = Auxiliary::radiansToAngle(CurrentToDistance - PreviousToCurrent);
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
    if (angle > 90) {
        angle = 180 - angle;
    }
    return std::pair<int, bool>{angle, clockwise};
}

std::pair<double, double> Auxiliary::GetMinMax(std::vector<double> &points) {
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::min();
    for (const auto &point: points) {
        if (min > point) {
            min = point;
        } else if (max < point) {
            max = point;
        }
    }
    return {min, max};
}

double Auxiliary::norm2d(double x, double y) {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

double
Auxiliary::GetMinDistance(const std::vector<Point> &points, const std::function<double(Point, Point)> &DistanceFunc) {
    double minDistance = std::numeric_limits<double>::max();
    for (const auto &point1: points) {
        for (const auto &point2: points) {
            double distance = DistanceFunc(point1, point2);
            if (distance != 0 && distance < minDistance) {
                minDistance = distance;
            }
        }
    }
    return minDistance;
}

std::pair<int, bool> Auxiliary::getRotationToTargetInFront(const Point &point1, const Point &point2) {
    Eigen::Vector3d vector1(point1.x, point1.y, 0);
    Eigen::Vector3d vector2(point2.x, point2.y, 0);
    Eigen::Vector3d unitVector1 = vector1 / vector1.norm();
    Eigen::Vector3d unitVector2 = vector2 / vector2.norm();
    int angle = int(radiansToAngle(acos(unitVector1.dot(unitVector2))));
    bool clockwise = unitVector1.cross(unitVector2).z() <= 0;
    std::cout << "getRotationToTargetInFront angle: " << angle << " clockwise: " << clockwise << std::endl;
    if (angle > 90) {
        angle = 180 - angle;
    }
    /*auto dot = point1.x * point2.x + point1.y * point2.y;
    auto det = point1.x * point2.y - point1.y * point2.x;
    int angle = std::ceil(Auxiliary::radiansToAngle(std::atan2(det, dot)));
    bool clockwise = angle > 0;
    std::cout << "getRotationToTargetInFront angle: " << angle << " clockwise: " << clockwise << std::endl;
    if (angle < 0) {
        angle *= -1;
    }
    if (angle > 90) {
        angle = 180 - angle;
        clockwise = !clockwise;
    }*/
    return std::pair<int, bool>{angle, clockwise};
}

long Auxiliary::myGcd(long a, long b) {
    if (a == 0)
        return b;
    else if (b == 0)
        return a;

    if (a < b)
        return myGcd(a, b % a);
    else
        return myGcd(b, a % b);
}

std::tuple<int, int, int> Auxiliary::getRationalInverse(double input) {
    double integral = std::floor(input);
    double frac = input - integral;

    const long precision = 1000000000; // This is the accuracy.

    long myGcd_ = myGcd(std::round(frac * precision), precision);

    long denominator = precision / myGcd_;
    long numerator = round(frac * precision) / myGcd_;

    std::cout << integral << " + ";
    std::cout << numerator << " / " << denominator << std::endl;
    return std::tuple(integral, numerator, denominator);
}

std::string Auxiliary::GetGeneralSettingsPath() {
    char currentDirPath[256];
    getcwd(currentDirPath, 256);
    std::string settingPath = currentDirPath;
    settingPath += "/../generalSettings.json";
    return settingPath;
}

double Auxiliary::calculateMeanOfDistanceDifferences(std::vector<double> distances) {
    double sumOfDistances = 0.0;
    for (int i = 0; i < distances.size() - 1; ++i) {
        sumOfDistances += std::abs(distances[i] - distances[i + 1]);
    }
    return sumOfDistances / (distances.size() - 1);
}

double Auxiliary::calculateVariance(const std::vector<double> &distances) {
    double sumOfDistances = 0.0;
    for (auto distance: distances) {
        sumOfDistances += distance;
    }
    double mean = sumOfDistances / distances.size();
    double variance = 0.0;
    for (auto distance: distances) {
        variance += pow(distance - mean, 2);
    }
    return variance;
}

cv::Mat Auxiliary::points3d_to_mat(const std::vector<cv::Point3d> &points3d) {
    std::size_t nPoints = points3d.size();
    cv::Mat mat((int) nPoints, 3, CV_64FC1);
    for (std::size_t i = 0; i < nPoints; i++) {
        mat.at<double>(i, 0) = points3d[i].x;
        mat.at<double>(i, 1) = points3d[i].y;
        mat.at<double>(i, 2) = points3d[i].z;
    }

    return mat.t();
}

std::pair<cv::Mat, cv::Mat> Auxiliary::calculateAlignMatrices(std::vector<cv::Point3d> points) {
    cv::Mat mu_align1;
    cv::Mat R_align;
    cv::reduce(points, mu_align1, 01, CV_REDUCE_AVG);

    cv::Point3d mu_align_pnt(mu_align1.at<double>(0), mu_align1.at<double>(1), mu_align1.at<double>(2));
    cv::Mat mu_align(mu_align_pnt);

    std::cout << "Centering points" << std::endl;
    for (auto &p: points) {
        p = p - mu_align_pnt;
    }

    cv::Mat A = Auxiliary::points3d_to_mat(points);
    cv::Mat w, u, vt;
    // cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    cv::SVDecomp(A, w, u, vt);
    R_align = u.t();


    return {R_align, mu_align};
}

std::vector<cv::Point3d> Auxiliary::convertToCvPoints(const std::vector<Point> &points) {
    std::vector<cv::Point3d> cvPoints;
    for (const auto &point: points) {
        cvPoints.emplace_back(cv::Point3d(point.x, point.y, point.z));
    }
    return cvPoints;
}

cv::Mat Auxiliary::convertPointToCVMat(const Point &point) {
    cv::Mat pnt3d(1, 3, CV_64FC1);
    pnt3d.at<double>(0, 0) = point.x;
    pnt3d.at<double>(0, 1) = point.y;
    pnt3d.at<double>(0, 2) = point.z;
    return pnt3d.t();
}

std::pair<cv::Mat, cv::Mat> Auxiliary::alignMap(std::vector<Point> &points) {
    auto[R_align, mu_align] = calculateAlignMatrices(convertToCvPoints(points));

    for (auto &point: points) {
        auto pnt3d = convertPointToCVMat(point);
        cv::Mat align_pos;
        align_pos = R_align * (pnt3d - mu_align);

        point.x = align_pos.at<double>(0, 0);
        point.y = align_pos.at<double>(1, 0);
        point.z = align_pos.at<double>(2, 0);
    }
    return {R_align, mu_align};
}
double Auxiliary::getAngleBySlopes(Line &line1, Line &line2) {
    Eigen::Vector3d vector1(1, line1.getSlope(), 0);
    Eigen::Vector3d vector2(1, line2.getSlope(), 0);
    Eigen::Vector3d unitVector1 = vector1 / vector1.norm();
    Eigen::Vector3d unitVector2 = vector2 / vector2.norm();
    return radiansToAngle(acos(unitVector1.dot(unitVector2)));
}

double Auxiliary::GetPitchFrom2Points(const Point &point1, const Point &point2) {
    return radiansToAngle(acos((point1.x * point2.x + point1.y * point2.y) /
                               (sqrt(pow(point1.x, 2) + pow(point1.y, 2)) *
                                sqrt(pow(point2.x, 2) + pow(point2.y, 2)))));
}

std::vector<double> Auxiliary::Get3dAnglesBetween2Points(const Point &point1, const Point &point2) {
    double x = acos((point1.y * point2.y + point1.z * point2.z) /
                    (sqrt(pow(point1.y, 2) + pow(point1.z, 2)) * sqrt(pow(point2.y, 2) + pow(point2.z, 2))));
    double y = acos((point1.x * point2.x + point1.z * point2.z) /
                    (sqrt(pow(point1.x, 2) + pow(point1.z, 2)) * sqrt(pow(point2.x, 2) + pow(point2.z, 2))));
    double z = acos((point1.x * point2.x + point1.y * point2.y) /
                    (sqrt(pow(point1.x, 2) + pow(point1.y, 2)) * sqrt(pow(point2.x, 2) + pow(point2.y, 2))));
    return {x, y, z};
}