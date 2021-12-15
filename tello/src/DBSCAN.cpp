#include "tello/include/DBSCAN.h"

int DBSCAN::run()
{
    int label = 1;
    std::vector<Point>::iterator iter;
    for(iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if ( iter->label == UNCLASSIFIED )
        {
            if ( expandCluster(*iter, label) != FAILURE )
            {
                label += 1;
            }
        }
    }

    return label;
}

int DBSCAN::expandCluster(Point point, int label)
{
    std::vector<int> clusterSeeds = calculateCluster(point);

    if ( clusterSeeds.size() < m_minPoints )
    {
        point.label = NOISE;
        return FAILURE;
    }
    else
    {
        int index = 0, indexCorePoint = 0;
        std::vector<int>::iterator iterSeeds;
        for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            Point cloudPoint = m_points[*iterSeeds];
            cloudPoint.label = label;
            if (cloudPoint.x == point.x && cloudPoint.y == point.y && cloudPoint.z == point.z )
            {
                indexCorePoint = index;
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);

        for( std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
        {
            std::vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

            if ( clusterNeighors.size() >= m_minPoints )
            {
                std::vector<int>::iterator iterNeighors;
                for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                {
                    auto neighborPoint = &m_points[*iterNeighors];
                    if ( neighborPoint->label == UNCLASSIFIED || neighborPoint->label == NOISE )
                    {
                        if ( neighborPoint->label == UNCLASSIFIED )
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        neighborPoint->label = label;
                    }
                }
            }
        }

        return SUCCESS;
    }
}

std::vector<int> DBSCAN::calculateCluster(const Point& point)
{
    int index = 0;
    std::vector<Point>::iterator iter;
    std::vector<int> clusterIndex;
    for( iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if ( Auxiliary::calculateDistance(point, *iter) <= m_epsilon )
        {
            clusterIndex.push_back(index);
        }
        index++;
    }
    return clusterIndex;
}


