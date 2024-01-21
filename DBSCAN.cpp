#include "DBSCAN.h"
float epsilonX = 50;
float epsilonY = 50;
float epsilonZ = 50;
/*
-------------------------------------------------------------
--------------------------- DBSCAN --------------------------
-------------------------------------------------------------
*/
std::map<int, std::vector<Point>> detect_objects_clustering(Point points[])
{

    // Calculate k-distance for each point, Calculate espilon
    calculateKDistance_set_Epsilon(points, epsilonX, epsilonY, epsilonZ);

    if (numPoints > 0)
    {
        // Apply DBSCAN for clustering only on valid points
        DBSCAN(points);
    }
    else
    {
        Serial.println("No valid data points collected.");
    }
    std::map<int, std::vector<Point>> clusterMap;

    // Populate cluster map with points
    for (int i = 0; i < numPoints; ++i)
    {
        int clusterId = points[i].clusterId;
        if (clusterId > 0)
        { // Assuming clusterId <= 0 are noise or unclassified
            clusterMap[clusterId].push_back(points[i]);
        }
    }
    return clusterMap;
}

int get_number_of_clusters(std::map<int, std::vector<Point>> clusterMap)
{
    return clusterMap.size();
}

bool arePointsNeighbors(const Point &p1, const Point &p2)
{

    return abs(p1.x - p2.x) <= epsilonX && abs(p1.y - p2.y) <= epsilonY && abs(p1.z - p2.z) <= epsilonZ;
}

void getNeighbors(Point points[], int pointIndex, std::vector<int> &neighbors)
{
    neighbors.clear();
    for (int i = 0; i < numPoints; ++i)
    {
        if (arePointsNeighbors(points[pointIndex], points[i]))
        {
            neighbors.push_back(i);
        }
    }
}

void expandCluster(Point points[], int pointIndex, int clusterId, std::vector<int> &neighbors)
{
    points[pointIndex].clusterId = clusterId;

    for (int i = 0; i < neighbors.size(); ++i)
    {
        int currNeighbor = neighbors[i];
        if (points[currNeighbor].clusterId == UNCLASSIFIED || points[currNeighbor].clusterId == NOISE)
        {
            points[currNeighbor].clusterId = clusterId;

            std::vector<int> newNeighbors;
            getNeighbors(points, currNeighbor, newNeighbors);

            for (int newNeighbor : newNeighbors)
            {
                // Check if new neighbor is already in neighbors vector
                if (std::find(neighbors.begin(), neighbors.end(), newNeighbor) == neighbors.end())
                {
                    neighbors.push_back(newNeighbor);
                }
            }
        }
    }
}

void DBSCAN(Point points[])
{
    int clusterId = 0;
    for (int i = 0; i < numPoints; ++i)
    {
        if (points[i].distance == 0 || points[i].distance == Limit_distance)
        {
            continue;
        }
        if (points[i].clusterId == UNCLASSIFIED)
        {
            std::vector<int> neighbors;
            getNeighbors(points, i, neighbors);
            if (neighbors.size() < minPoints)
            {
                points[i].clusterId = NOISE;
            }
            else
            {
                expandCluster(points, i, ++clusterId, neighbors);
            }
        }
    }
}