//
// Created by rbdstudent on 27/06/2021.
//

#include "include/Polygon.h"
#include "include/thoretic.h"
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


void call_to_python_program(std::string data_path, int coreset_size, std::string output_path){
    std::string arguments = data_path;
    arguments  += " " + std::to_string(coreset_size) + " "; // coreset size  // TODO : change it it to argument
    arguments  += output_path;

    std::string fullCommand = "/home/livne/CLionProjects/callToPythonProgram/main " + arguments;

    int result = system(fullCommand.c_str());
    if (result == 0)
    {
        std::cout << "Command executed successfully." << std::endl;
    }
    else
    {
        std::cout << "Command execution failed." << std::endl;
    }
}


std::vector<int> read_coreset_indexes_from_file(std::string path){
    std::vector<int> indexes;

    std::ifstream file(path);
    if (file.is_open())
    {
        int index;
        while (file >> index)
        {
            indexes.push_back(index);
        }
        file.close();
    }
    else
    {
        std::cout << "Failed to open file." << std::endl;
    }
    return indexes;
}


std::vector<Point> extractPoints(const std::vector<Point>& points, const std::vector<int>& indexes)
{
    std::vector<Point> extractedPoints;
    for (int index : indexes) {
        if (index >= 0 && index < points.size()) {
            extractedPoints.push_back(points[index]);
        }
    }
    return extractedPoints;
}


void rectangle_main() {
    std::string datasetFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/datasets/buildings/newLab/pointData.csv";
    auto points = getPointsFromFile(datasetFilePath);
    auto start = std::chrono::high_resolution_clock::now();
    call_to_python_program(datasetFilePath, 50,
                           "/home/livne/CLionProjects/callToPythonProgram/output.txt");
    auto indexes = read_coreset_indexes_from_file("/home/livne/CLionProjects/callToPythonProgram/output.txt");
    auto coreset_points = extractPoints(points, indexes);

    auto stop1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::seconds>(stop1 - start);
    std::cout << "Call to coreset take " << duration1.count() << " seconds" << std::endl;

    thoretic obj(coreset_points);
    auto rectangle = obj.getOptimalRectangle(coreset_points);
    auto rect_vertices = obj.getVerticesOfRectangle(rectangle);

    auto stop2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::seconds>(stop2 - stop1);
    std::cout << "Calculate rectangle take " << duration2.count() << " seconds" << std::endl;

    Auxiliary::showCloudPointAndCoreset(rect_vertices, points, coreset_points);

    auto currentPosition = Auxiliary::GetCenterOfMass(points);
    auto navigationPoints = obj.getExitPointsByRectangle(rect_vertices, points, currentPosition, true);
    //plot_for_dan();
}


void polygon_main() {
    //std::string datasetFilePath = Auxiliary::GetDataSetsDirPath() + "pointData1234.csv";
    // std::string datasetFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/datasets/buildings/RoomsDatabase/dani_office/pointData0.csv";
    std::string datasetFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/datasets/pointData1.csv";
    auto points = getPointsFromFile(datasetFilePath);
    /*std::string datasetFilePath1 = Auxiliary::GetDataSetsDirPath() + "pointData1000.csv";
    auto points1 = getPointsFromFile(datasetFilePath1);
    matplotlibcpp::scatter(Auxiliary::getXValues(points1), Auxiliary::getYValues(points1), 3);*/
    //matplotlibcpp::scatter(Auxiliary::getXValues(points), Auxiliary::getYValues(points), 3);
    //matplotlibcpp::show();
    auto start = std::chrono::high_resolution_clock::now();
    //auto points = getPointsFromFile(datasetFilePath);
    Polygon polygon(points, Point(), true);
    auto vertex = polygon.getExitPointsByPolygon(true);
    //plot_for_dan();
}


// Function to generate a rectangle and sample points near it with Gaussian noise
std::vector<Point> generatePointsWithNoise(int samplesNumber, double noiseStdDev) {
    // define random generators
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // noise generator
    std::mt19937 gen(seed);
    std::normal_distribution<double> noise_dis(0, noiseStdDev);
    // defining points generator
    std::uniform_real_distribution<double> defining_dis(-10, 10);

    std::vector<Point> points_define_rect;
    for (int i=0; i<5; i++){
        double x = defining_dis(gen);
        double y = defining_dis(gen);
        points_define_rect.emplace_back(Point(x,y,0));
    }
    thoretic object(points_define_rect);
    auto rect_vertices = object.getVerticesOfRectangle(points_define_rect);
    std::vector<Point> points;
    for (int j=0; j<4; j++) {
        auto vertex1 = rect_vertices[j];
        auto vertex2 = rect_vertices[(j+1)%4];
        double slope = (vertex1.y - vertex2.y) / (vertex1.x - vertex2.x);
        double y_intercept = vertex1.y - vertex1.x * slope;
        double upperBound = vertex1.x > vertex2.x ? vertex1.x : vertex2.x;
        double lowerBound = vertex1.x <= vertex2.x ? vertex1.x : vertex2.x;
        // Random number generation
        std::uniform_real_distribution<double> dis(lowerBound-2, upperBound+2);
        for (int i = 0; i < samplesNumber / 4; i++) {
            // Generate a random number
            double random_x = dis(gen);
            double noise = noise_dis(gen);
            double noised_y = random_x * slope + y_intercept + noise;
            points.emplace_back(Point(random_x, noised_y, 0));
        }
    }

    // write points to file
    std::ofstream pointData;
    pointData.open("/tmp/pointDataTemp.csv");
    for (auto p: points) {
        pointData << p.x << "," << p.y << "," << p.z << std::endl; // TODO : IMPORTANT! need to check if it's appropriate python callee
    }
    pointData.close();

    return points;
}


void costs_for_different_coreset_sizes(){
    auto points = generatePointsWithNoise(1000, 0.3);
    Auxiliary::showCloudPoint(points);

    std::string outputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/output.txt";

    std::ofstream pointData;
    pointData.open("/home/livne/Documents/costs.txt");

    for(int i=10; i<26; i++) {
        std::cout << "----------------------------------- " << i << " -----------------------------------" << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        call_to_python_program("/tmp/pointDataTemp.csv", i, outputFilePath);
        auto indexes = read_coreset_indexes_from_file(outputFilePath);
        auto coreset_points = extractPoints(points, indexes);

        auto stop1 = std::chrono::high_resolution_clock::now();
        auto duration1 = std::chrono::duration_cast<std::chrono::seconds>(stop1 - start);

        thoretic obj(coreset_points);
        auto rectangle = obj.getOptimalRectangle(coreset_points);
        auto rect_vertices = obj.getVerticesOfRectangle(rectangle);
        auto cost = obj.calculateDistanceSum(points, rectangle);
        pointData << cost << std::endl;

        auto stop2 = std::chrono::high_resolution_clock::now();
        auto duration2 = std::chrono::duration_cast<std::chrono::seconds>(stop2 - stop1);
        std::cout << "Calculate rectangle take " << duration2.count() << " seconds" << std::endl;
    }
    pointData.close();
}


void running_times_for_different_coreset_sizes(){
    auto points = generatePointsWithNoise(1000, 0.3);
    Auxiliary::showCloudPoint(points);

    std::string outputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/output.txt";

    std::ofstream pointData;
    pointData.open("/home/livne/Documents/times.txt");

    for(int i=10; i<26; i++) {
        std::cout << "----------------------------------- " << i << " -----------------------------------" << std::endl;
        call_to_python_program("/tmp/pointDataTemp.csv", i, outputFilePath);
        auto indexes = read_coreset_indexes_from_file(outputFilePath);
        auto coreset_points = extractPoints(points, indexes);

        thoretic obj(coreset_points);

        auto start = std::chrono::high_resolution_clock::now();
        auto rectangle = obj.getOptimalRectangle(coreset_points);
        auto stop = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        pointData << duration.count() << std::endl;

        std::cout << "Calculate rectangle take " << duration.count() << " seconds" << std::endl;
    }
    pointData.close();
}


int main() {
    // polygon_main();
    // costs_for_different_coreset_sizes();
    running_times_for_different_coreset_sizes();
    //Auxiliary::showCloudPointAndCoreset(rect_vertices, points, coreset_points);
    return 0;
}