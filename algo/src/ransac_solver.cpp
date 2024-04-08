#include <iostream>
#include <vector>
#include <random>
#include <unordered_set>
#include "include/Point.h"
#include "include/thoretic.h"

// Define a function to calculate the distance between a point and the model
/*double distance(std::vector<Line> rect_lines, const Point& point) {
    // Here, calculate the distance between the point (x, y) and the model
    double min_distance = std::numeric_limits<double>::max();
    for (const Line& line : rect_lines) {
        double distance = line.getDistanceToPoint(point);
        min_distance = std::min(min_distance, distance);
    }
    return min_distance;
}

// RANSAC algorithm
std::vector<Point> RANSAC(const std::vector<Point>& points, int maxIterations, double threshold) {
    int n = points.size();
    int bestInliers = 0;
    std::vector<Point> bestModel;

    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < maxIterations; ++i) {
        // Randomly select two points from the data
        std::uniform_int_distribution<int> dist(0, n - 1);
        int idx1 = dist(gen);
        int idx2 = dist(gen);
        int idx3 = dist(gen);
        int idx4 = dist(gen);
        int idx5 = dist(gen);
        std::vector<Point> defining_points = {points[idx1], points[idx2], points[idx3], points[idx4], points[idx5]};

        // Fit a model to the selected points
        Line line1(defining_points[0], defining_points[1]);
        auto slope = line1.slope;
        Line line2(defining_points[2], slope);
        Line line3(defining_points[3], -1 / slope);
        Line line4(defining_points[4], -1 / slope);
        std::vector<Line> rect_lines = {line1, line2, line3, line4};

        // Count the inliers (data points close enough to the model)
        int inliers = 0;
        for (int j = 0; j < n; ++j) {
            if (distance(rect_lines, points[j]) < threshold) {
                inliers++;
            }
        }

        // Check if the current model is better than the previous best model
        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestModel = defining_points;

            // Update the number of iterations to reduce the probability of selecting outliers
            double outlierRatio = 1.0 - static_cast<double>(inliers) / n;
            double probabilityNoOutliers = 1.0 - outlierRatio * outlierRatio;
            //maxIterations = std::min(maxIterations, static_cast<int>(std::log(1.0 - 0.99) / std::log(probabilityNoOutliers)));
            std::cout << maxIterations << std::endl;
        }
    }
    return bestModel;
}*/


int countInliers(const std::vector<Point>& points, const std::vector<Point>& rect, double threshold) {
    int counter = 0;
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
        if (min_distance <= threshold) {
            counter++;
        }
    }
    return counter;
}


std::vector<Point> ransac_loop(std::vector<Point> input_points, double threshold) {
    std::vector<Point> optimal_rect(5);
    int max_inliers = 0;

    // Iterate over every combination of 5 points  //
    for (size_t i = 0; i < input_points.size() - 4; ++i) {
        for (size_t j = i + 1; j < input_points.size() - 3; ++j) {
            for (size_t k = j + 1; k < input_points.size() - 2; ++k) {
                for (size_t l = k + 1; l < input_points.size()-1; ++l) {
                    for (size_t m = l + 1; m < input_points.size(); ++m) {
                        std::vector<unsigned long> indices = {i, j, k, l, m};
                        do{
                            std::vector<Point> rect = {input_points[indices[0]], input_points[indices[1]], input_points[indices[2]], input_points[indices[3]], input_points[indices[4]]};
                            int inliers = countInliers(input_points, rect, threshold);
                            if (inliers > max_inliers){
                                max_inliers = inliers;
                                optimal_rect = rect;
                            }
                        } while(std::next_permutation(indices.begin(), indices.end()));
                    }
                }
            }
        }
    }
    return optimal_rect;
}


std::vector<Point> ransac_loop_early_termination(std::vector<Point> input_points, double threshold, double inliers_percent) {
    std::vector<Point> optimal_rect(5);
    int max_inliers = 0;
    int max_iterations = sqrt(input_points.size()) * log(1-0.99) / log(1- pow(inliers_percent, 5));
    int counter = 0;
    // shuffle the data
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 g(seed);
    std::shuffle(input_points.begin(), input_points.end(), g);

    // Iterate over every combination of 5 points  //
    for (size_t i = 0; i < input_points.size() - 4; ++i) {
        for (size_t j = i + 1; j < input_points.size() - 3; ++j) {
            for (size_t k = j + 1; k < input_points.size() - 2; ++k) {
                for (size_t l = k + 1; l < input_points.size()-1; ++l) {
                    for (size_t m = l + 1; m < input_points.size(); ++m) {
                        std::vector<unsigned long> indices = {i, j, k, l, m};
                        do{
                            std::vector<Point> rect = {input_points[indices[0]], input_points[indices[1]], input_points[indices[2]], input_points[indices[3]], input_points[indices[4]]};
                            int inliers = countInliers(input_points, rect, threshold);
                            if (inliers > max_inliers){
                                max_inliers = inliers;
                                optimal_rect = rect;
                            }
                        } while(std::next_permutation(indices.begin(), indices.end()));
                    }
                    if (counter == max_iterations)  // stop condition : early termination
                        return optimal_rect;
                    counter++;
                }
            }
        }
    }
    return optimal_rect;
}


/*std::vector<Point> ransac_loop(std::vector<Point> input_points, double threshold) {
    std::vector<Point> optimal_rect(5);
    int max_inliers = 0;
    int counter = 0;

    // Create a random number generator engine
    std::mt19937 gen(std::random_device{}());
    // Define the range of integers you want to sample from
    int lower_bound = 0;
    int upper_bound = input_points.size();
    // Create a uniform integer distribution with the desired range
    std::uniform_int_distribution<int> dist(lower_bound, upper_bound);
    // Iterate over every combination of 5 points  //
    while (counter < std::pow(input_points.size(),5)) {
        std::unordered_set<int> sampled_numbers;
        // Sample 5 unique numbers
        while (sampled_numbers.size() < 5) {
            int sampled_number = dist(gen);
            sampled_numbers.insert(sampled_number);
        }
        // Convert the unordered set to a vector (optional)
        std::vector<int> indices(sampled_numbers.begin(), sampled_numbers.end());

        int max_inliers_permutation = 0;
        std::vector<Point> optimal_rect_permutation(5);
        do {
            std::vector<Point> rect = {input_points[indices[0]], input_points[indices[1]], input_points[indices[2]],
                                       input_points[indices[3]], input_points[indices[4]]};
            int inliers = countInliers(input_points, rect, threshold);
            if (inliers > max_inliers_permutation) {
                max_inliers_permutation = inliers;
                optimal_rect_permutation = rect;
            }
        } while (std::next_permutation(indices.begin(), indices.end()));
        if (max_inliers_permutation > max_inliers) {
            std::cout << max_inliers_permutation << std::endl;
            *if (max_inliers_permutation < 1.01 * max_inliers) { // converge
                return optimal_rect_permutation;
            }
            max_inliers = max_inliers_permutation;
            optimal_rect = optimal_rect_permutation;
        }
        counter += 1;
    }
    std::cout << "return" << std::endl;
    return optimal_rect;
}*/


// Function to generate a rectangle and sample points near it with Gaussian noise
std::vector<Point> generatePointsWithNoise(int samplesNumber, double noiseStdDev, double outliers_percent, unsigned seed) {
    // noise generator
    std::mt19937 gen(seed);
    std::normal_distribution<double> noise_dis(0, noiseStdDev);
    std::normal_distribution<double> very_noise_dis(0, 10);
    // defining points generator
    std::uniform_real_distribution<double> defining_dis(-10, 10);

    int outliers_amount = outliers_percent * samplesNumber;
    int no_outliers_amount = samplesNumber - outliers_amount;

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
        std::uniform_real_distribution<double> dis(lowerBound, upperBound);  // TODO : if we want tails, add +-2
        for (int i = 0; i < no_outliers_amount / 4; i++) {
            // Generate a random number
            double random_x = dis(gen);
            double noise = noise_dis(gen);
            double noised_y = random_x * slope + y_intercept + noise;
            points.emplace_back(Point(random_x, noised_y, 0));
        }
        for (int i = 0; i < outliers_amount / 4; i++) {
            // Generate a random number
            double random_x = dis(gen);
            double noise = very_noise_dis(gen);
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

