//
// Created by rbdstudent on 27/06/2021.
//

#include "include/Polygon.h"
#include "include/thoretic.h"
#include "src/ransac_solver.cpp"
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


void call_to_python_program(std::string data_path, int coreset_size, std::string output_path, std::string weights_output_path){
    std::string arguments = data_path;
    arguments  += " " + std::to_string(coreset_size) + " "; // coreset size  // TODO : change it it to argument
    arguments  += output_path + " ";
    arguments  += weights_output_path;

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
    std::string weightsOutputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/weights_output.txt";

    auto points = getPointsFromFile(datasetFilePath);
    auto start = std::chrono::high_resolution_clock::now();
    call_to_python_program(datasetFilePath, 50,
                           "/home/livne/CLionProjects/callToPythonProgram/output.txt", weightsOutputFilePath);
    auto indexes = read_coreset_indexes_from_file("/home/livne/CLionProjects/callToPythonProgram/output.txt");
    auto weights = read_coreset_indexes_from_file(weightsOutputFilePath);
    auto coreset_points = extractPoints(points, indexes);

    auto stop1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::seconds>(stop1 - start);
    std::cout << "Call to coreset take " << duration1.count() << " seconds" << std::endl;

    thoretic obj(coreset_points);
    auto rectangle = obj.getOptimalRectangle(coreset_points, weights);
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


void costs_for_different_coreset_sizes(){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto points = generatePointsWithNoise(1000, 0.3, 0.2, seed);
    Auxiliary::showCloudPoint(points);

    std::string outputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/output.txt";
    std::string weightsOutputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/weights_output.txt";

    std::ofstream pointData;
    pointData.open("/home/livne/Documents/costs.txt");

    for(int i=10; i<26; i++) {
        std::cout << "----------------------------------- " << i << " -----------------------------------" << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        call_to_python_program("/tmp/pointDataTemp.csv", i, outputFilePath, weightsOutputFilePath);
        auto indexes = read_coreset_indexes_from_file(outputFilePath);
        auto weights = read_coreset_indexes_from_file(weightsOutputFilePath);
        auto coreset_points = extractPoints(points, indexes);

        auto stop1 = std::chrono::high_resolution_clock::now();
        auto duration1 = std::chrono::duration_cast<std::chrono::seconds>(stop1 - start);

        thoretic obj(coreset_points);
        auto rectangle = obj.getOptimalRectangle(coreset_points, weights);
        auto rect_vertices = obj.getVerticesOfRectangle(rectangle);
        auto cost = obj.calculateDistanceSum(points, rectangle, weights);
        pointData << cost << std::endl;

        auto stop2 = std::chrono::high_resolution_clock::now();
        auto duration2 = std::chrono::duration_cast<std::chrono::seconds>(stop2 - stop1);
        std::cout << "Calculate rectangle take " << duration2.count() << " seconds" << std::endl;
    }
    pointData.close();
}


void running_times_for_different_coreset_sizes(){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto points = generatePointsWithNoise(1000, 0.3, 0.1, seed);
    Auxiliary::showCloudPoint(points);

    std::string outputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/output.txt";
    std::string weightsOutputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/weights_output.txt";

    std::ofstream pointData;
    pointData.open("/home/livne/Documents/times.txt");

    for(int i=10; i<26; i++) {
        std::cout << "----------------------------------- " << i << " -----------------------------------" << std::endl;
        call_to_python_program("/tmp/pointDataTemp.csv", i, outputFilePath, weightsOutputFilePath);
        auto indexes = read_coreset_indexes_from_file(outputFilePath);
        auto weights = read_coreset_indexes_from_file(weightsOutputFilePath);
        auto coreset_points = extractPoints(points, indexes);

        thoretic obj(coreset_points);

        auto start = std::chrono::high_resolution_clock::now();
        auto rectangle = obj.getOptimalRectangle(coreset_points, weights);
        auto rect_vertices = obj.getVerticesOfRectangle(rectangle);
        Auxiliary::showCloudPointAndCoreset(rect_vertices, points, coreset_points);
        auto stop = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        pointData << duration.count() << std::endl;

        std::cout << "Calculate rectangle take " << duration.count() << " seconds" << std::endl;
    }
    pointData.close();
}


double calculateCost(const std::vector<Point>& points, const std::vector<Point>& rect) {
    double sum = 0.0;

    Line line1(rect[0], rect[1]);
    auto slope = line1.slope;
    Line line2(rect[2], slope);
    Line line3(rect[3], -1 / slope);
    Line line4(rect[4], -1 / slope);

    std::vector<Line> rect_lines = {line1, line2, line3, line4};
    for (const Point& point : points) {
        double min_distance = std::numeric_limits<double>::max();
        for (const Line& line : rect_lines) {
            double distance = line.getDistanceToPoint(point);
            min_distance = std::min(min_distance, distance);
        }
        sum += min_distance;
    }
    return sum / points.size();
}


void running_times_for_different_input_sizes(double k){
    std::string outputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/output.txt";
    std::string weightsOutputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/weights_output.txt";

    for (int i=0; i<20; i++) {

        std::ofstream pointData1; // times for our algorithm
        std::ofstream pointData2; // times for ransac + coreset
        std::ofstream pointData3; // costs for our algorithm
        std::ofstream pointData4; // costs for ransac + coreset
        std::ofstream pointData5; // times for ransac + uniform sampling
        std::ofstream pointData6; // costs for ransac + uniform sampling
        pointData1.open("/home/livne/Documents/graphs/times_as_function_of_input_size/our_algorithm/k=" +
                        std::to_string(k) + "_" + std::to_string(i) + ".txt");
        pointData2.open("/home/livne/Documents/graphs/times_as_function_of_input_size/ransac_algorithm_coreset/k=" +
                        std::to_string(k) + "_" + std::to_string(i) + ".txt");
        pointData3.open("/home/livne/Documents/graphs/costs_as_function_of_input_size/our_algorithm/k=" +
                        std::to_string(k) + "_" + std::to_string(i) + ".txt");
        pointData4.open("/home/livne/Documents/graphs/costs_as_function_of_input_size/ransac_algorithm_coreset/k=" +
                        std::to_string(k) + "_" + std::to_string(i) + ".txt");
        pointData5.open("/home/livne/Documents/graphs/times_as_function_of_input_size/ransac_algorithm_uniform/k=" +
                        std::to_string(k) + "_" + std::to_string(i) + ".txt");
        pointData6.open("/home/livne/Documents/graphs/costs_as_function_of_input_size/ransac_algorithm_uniform/k=" +
                        std::to_string(k) + "_" + std::to_string(i) + ".txt");

        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        for (int size = 100; size <= 1000; size += 50) {
            std::cout << "----------------------------------- iteration: " << i+1 << ", input size: " << size <<
                      " -----------------------------------" << std::endl;
            auto points = generatePointsWithNoise(size, 0.3, k, seed);
            // our algorithm
            auto start = std::chrono::high_resolution_clock::now();
            call_to_python_program("/tmp/pointDataTemp.csv", sqrt(size), outputFilePath, weightsOutputFilePath);
            auto indexes = read_coreset_indexes_from_file(outputFilePath);
            auto weights = read_coreset_indexes_from_file(weightsOutputFilePath);
            auto coreset_points = extractPoints(points, indexes);

            thoretic obj(coreset_points);
            auto rectangle = obj.getOptimalRectangle(coreset_points, weights);
            auto stop = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            pointData1 << duration.count() << std::endl;

            auto cost = calculateCost(points, rectangle);
            pointData3 << cost << std::endl;

            // ransac algorithm + coreset
            /*start = std::chrono::high_resolution_clock::now();
            call_to_python_program("/tmp/pointDataTemp.csv", sqrt(points.size()), outputFilePath, weightsOutputFilePath);
            indexes = read_coreset_indexes_from_file(outputFilePath);
            weights = read_coreset_indexes_from_file(weightsOutputFilePath);
            coreset_points = extractPoints(points, indexes);

            double threshold = 1;
            auto bestModel = ransac_loop(coreset_points, threshold);

            stop = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            pointData2 << duration.count() << std::endl;

            cost = calculateCost(points, bestModel);
            pointData4 << cost << std::endl;*/

            start = std::chrono::high_resolution_clock::now();

            double threshold = 1;
            auto bestModel = ransac_loop_early_termination(points, threshold, 1-k);

            stop = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            pointData2 << duration.count() << std::endl;

            cost = calculateCost(points, bestModel);
            pointData4 << cost << std::endl;

            // ransac algorithm + uniform sampling
            start = std::chrono::high_resolution_clock::now();

            int lower_bound = 0;
            int upper_bound = sqrt(points.size());
            // Create a random number generator engine
            std::mt19937 gen(std::random_device{}());
            // Create a uniform integer distribution with the desired range
            std::uniform_int_distribution<int> dist(lower_bound, upper_bound);
            std::unordered_set<int> sampled_indexes;
            while (sampled_indexes.size() < sqrt(points.size())) {
                int sampled_number = dist(gen);
                sampled_indexes.insert(sampled_number);
            }
            // Convert the unordered set to a vector (optional)
            std::vector<int> indices(sampled_indexes.begin(), sampled_indexes.end());

            coreset_points = extractPoints(points, indices);

            threshold = 1;
            bestModel = ransac_loop(coreset_points, threshold);

            stop = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            pointData5 << duration.count() << std::endl;

            cost = calculateCost(points, bestModel);
            pointData6 << cost << std::endl;
        }
        pointData1.close();
        pointData2.close();
        pointData3.close();
        pointData4.close();
        pointData5.close();
        pointData6.close();
    }
}


int ransac_runner(double inliers_percent) {
    std::string outputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/output.txt";
    std::string weightsOutputFilePath = "/home/livne/CLionProjects/AutonomousDroneCPP_master/exe/weightsOutput.txt";
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto points = generatePointsWithNoise(1000, 0.3, 1-inliers_percent, seed);
    /*call_to_python_program("/tmp/pointDataTemp.csv", sqrt(points.size()), outputFilePath, weightsOutputFilePath);
    auto indexes = read_coreset_indexes_from_file(outputFilePath);
    auto weights = read_coreset_indexes_from_file(weightsOutputFilePath);
    auto coreset_points = extractPoints(points, indexes);*/

    double threshold = 1;
    auto start = std::chrono::high_resolution_clock::now();

    auto bestModel = ransac_loop_early_termination(points, threshold, inliers_percent);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;

    thoretic obj(points);
    auto rect_vertices = obj.getVerticesOfRectangle(bestModel);
    double cost = calculateCost(points, bestModel);
    std::cout << cost << std::endl;
    Auxiliary::showCloudPointAndCoreset(rect_vertices, points, points);

    return 0;
}


int main() {
    // rectangle_main();
    std::cout << "find me : k=0" << std::endl;
    running_times_for_different_input_sizes(0.01);
    std::cout << "find me : k=0.1" << std::endl;
    running_times_for_different_input_sizes(0.1);
    std::cout << "find me : k=0.5" << std::endl;
    running_times_for_different_input_sizes(0.5);
    // ransac_runner(0.99);
    //Auxiliary::showCloudPointAndCoreset(rect_vertices, points, coreset_points);
    return 0;
}