
#include <iostream>
#include "dbscan.h"

int DBSCAN::run()
{
    std::cout << "RUN <DBSCAN> \n";
     std::cout << "[DBSCAN:m_minPoints] " << m_minPoints << std::endl;
    std::cout << "[DBSCAN:m_epsilon] " << m_epsilon << std::endl;
    std::cout << "[DBSCAN:m_pointSize] " << m_pointSize << std::endl;
    std::cout << "[DBSCAN:m_points[0]] " << m_points[0].clusterID << std::endl;
    int clusterID = 1;
    vector<Point>::iterator iter;
    for(iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        std::cout << iter->clusterID << std::endl;
        if ( iter->clusterID == UNCLASSIFIED )
        {
            if ( expandCluster(*iter, clusterID) != FAILURE )
            {
                clusterID += 1;
                std::cout << clusterID << std::endl;
            }
            else{
                std::cout << "FAILURE <DBSCAN> \n";
            }
        }
    }

    std::cout << "END <DBSCAN> \n";
    return 0;
}

int DBSCAN::expandCluster(Point point, int clusterID)
{
    std::cout << "1\n";
    vector<int> clusterSeeds = calculateCluster(point);
    
    std::cout << "2\n";
    if ( clusterSeeds.size() < m_minPoints )
    {
        point.clusterID = NOISE;
        return FAILURE;
    }
    else
    {
        std::cout << "3\n";
        int index = 0, indexCorePoint = 0;
        vector<int>::iterator iterSeeds;
        for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            m_points.at(*iterSeeds).clusterID = clusterID;
            if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z )
            {
                indexCorePoint = index;
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);

        std::cout << "4\n";
        
        std::cout << "[clusterSeeds] " << clusterSeeds.size() << std::endl;
        for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
        {
            std::cout << "[i] " <<i << " [n] " << n << std::endl;
            vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));
            

            // std::cout << "[clusterNeighors] " << clusterNeighors.size() << std::endl;
            if ( clusterNeighors.size() >= m_minPoints )
            {
                vector<int>::iterator iterNeighors;
                
                for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                {
                    
                    // std::cout << "[m_points.at(*iterNeighors).clusterID] "<< m_points.at(*iterNeighors).clusterID << std::endl;
                    // std::cout << "6\n";
                    if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE )
                    {
                        if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED )
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points.at(*iterNeighors).clusterID = clusterID;
                    }
                }
            }
        }

        std::cout << "sssss\n";

        return SUCCESS;
    }
}

vector<int> DBSCAN::calculateCluster(Point point)
{
    int index = 0;
    vector<Point>::iterator iter;
    vector<int> clusterIndex;
    for( iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if ( calculateDistance(point, *iter) <= m_epsilon )
        {
            clusterIndex.push_back(index);
        }
        index++;
    }
    return clusterIndex;
}

inline double DBSCAN::calculateDistance(const Point& pointCore, const Point& pointTarget )
{
    return pow(pointCore.x - pointTarget.x,2)+pow(pointCore.y - pointTarget.y,2)+pow(pointCore.z - pointTarget.z,2);
}


