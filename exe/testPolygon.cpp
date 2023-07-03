//
// Created by rbdstudent on 27/06/2021.
//

#include "include/Polygon.h"
#include "include/thoretic.h"
//#include <tqdm/tqdm.h>
#include<fstream>
#include <chrono>

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


void divide_points(std::vector<Point> basePoints, std::vector<Point> points){
    std::vector<std::vector<Point>> clouds;
    for (int i=0; i<basePoints.size(); i++)
        clouds.emplace_back(std::vector<Point>{});
    for (auto point : points){
        int minDistance = 1000000000;
        int minIndex = -1;
        for (int i=0; i<basePoints.size(); i++) {
            auto distance = Auxiliary::calculateDistanceXY(point, basePoints[i]);
            if (distance < minDistance) {
                minDistance = distance;
                minIndex = i;
            }
        }
        clouds[minIndex].emplace_back(point);
    }
    // TODO : important! think if combine in BFS logic
    int angle = 30;
    double ratio = 0.4;
    matplotlibcpp::clf();
    // show merged clouds
    for (auto cloud : clouds) {
        matplotlibcpp::scatter(Auxiliary::getXValues(cloud), Auxiliary::getYValues(cloud), 2.0);
        Polygon polygon(cloud, Point());
        auto vertex = polygon.getExitPointsByPolygon(false, true, angle, ratio);
        matplotlibcpp::plot(Auxiliary::getXValues(vertex), Auxiliary::getYValues(vertex));
    }
    matplotlibcpp::scatter(Auxiliary::getXValues(basePoints), Auxiliary::getYValues(basePoints), 30.0);
    matplotlibcpp::show();
    // show each cloud separately with each next navigation points
    for (auto cloud : clouds) {
        matplotlibcpp::scatter(Auxiliary::getXValues(cloud), Auxiliary::getYValues(cloud), 2.0);
        Polygon polygon(cloud, Point());
        auto vertex = polygon.getExitPointsByPolygon(false, true, angle, ratio);
        matplotlibcpp::plot(Auxiliary::getXValues(vertex), Auxiliary::getYValues(vertex));
        auto navigationPoints = polygon.getExitPointsByPolygon(false, false, angle, ratio);
        matplotlibcpp::scatter(Auxiliary::getXValues(navigationPoints), Auxiliary::getYValues(navigationPoints), 30);
        matplotlibcpp::show();
    }
}


void plot_for_dan(){
    std::string datasetFilePathBase = Auxiliary::GetDataSetsDirPath() + "pointData0.csv";
    std::string datasetFilePath = Auxiliary::GetDataSetsDirPath() + "pointDataExtended.csv";
    auto points = getPointsFromFile(datasetFilePath);
    auto basePoints = getPointsFromFile(datasetFilePathBase);
    Polygon polygon(basePoints, Point());
    auto vertex = polygon.getExitPointsByPolygon(false);
    divide_points(vertex, points);
}



int main() {
    std::string datasetFilePath = Auxiliary::GetDataSetsDirPath() + "pointData1234.csv";
    auto points = getPointsFromFile(datasetFilePath);
    /*std::string datasetFilePath1 = Auxiliary::GetDataSetsDirPath() + "pointData1000.csv";
    auto points1 = getPointsFromFile(datasetFilePath1);
    matplotlibcpp::scatter(Auxiliary::getXValues(points1), Auxiliary::getYValues(points1), 3);*/
    matplotlibcpp::scatter(Auxiliary::getXValues(points), Auxiliary::getYValues(points), 3);
    matplotlibcpp::show();
    auto start = std::chrono::high_resolution_clock::now();
    //auto points = getPointsFromFile(datasetFilePath);
    //Polygon polygon(points, Point(), true); // TODO : remove comment
    // auto vertex = polygon.getExitPointsByPolygon(true); // TODO : remove comment
    std::vector<Point> firstTen(points.begin(), points.begin() + 50);
    thoretic obj(firstTen);
    auto rectangle = obj.getOptimalRectangle(firstTen);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << " microseconds" << std::endl;
    //plot_for_dan();
}