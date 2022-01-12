//
// Created by rbdstudent on 27/06/2021.
//

#include "include/Polygon.h"
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
        lineStream >> point.qx;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qy;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qz;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.qw;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.frameId;
        points.push_back(point);
    }
    return points;

}

int main() {
    std::vector<Point> points = getPointsFromFile("/tmp/pointData1.csv");
    auto start = std::chrono::high_resolution_clock::now();
    Polygon polygon(points, Point());
    polygon.getExitPointsByPolygon(true);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;
}