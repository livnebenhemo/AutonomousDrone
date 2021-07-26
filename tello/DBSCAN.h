#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include "Point.h"
#include "Auxiliary.h"
#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

class DBSCAN {
public:
    DBSCAN(unsigned int minPts, float eps, std::vector<Point> points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
    }
    ~DBSCAN(){}

    int run();
    std::vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    std::vector<Point> getPoints(){return m_points;}
    void setMinPoints(int newMinPoints){m_minPoints = newMinPoints;}
private:
    std::vector<Point> m_points;
    unsigned int m_minPoints;
    float m_epsilon;
};

#endif // DBSCAN_H