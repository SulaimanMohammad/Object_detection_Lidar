#include "ClusterInfo.h"

/*
----------------------------------------------------------------
---------------------- Info of clusters ------------------------
----------------------------------------------------------------
*/

std::vector<ClusterInfo> gather_clusters_info(Point points[], int numberOfClusters)
{
    std::vector<ClusterInfo> clusters;
    // clusterData array is sized to numberOfClusters + 1 to accommodate the fact that clusterId starts at 1
    ClusterInfo clusterData[numberOfClusters + 1];

    // Initialize cluster data
    for (int i = 0; i <= numberOfClusters; ++i)
    {
        clusterData[i] = {0, 0, 0,
                          0, 0, 0,
                          0,
                          0,
                          -1, 0, 0,
                          Limit_distance, -1, 0, 0, 0, 0}; // Initialize minDistance with FLT_MAX
    }

    // Iterating through each point
    for (int i = 0; i < numPoints; ++i)
    {
        int clusterId = points[i].clusterId;
        if (clusterId > 0 && clusterId <= numberOfClusters)
        {
            // Accumulate for centroid calculation
            clusterData[clusterId].sumX += points[i].x;
            clusterData[clusterId].sumY += points[i].y;
            clusterData[clusterId].sumZ += points[i].z;
            clusterData[clusterId].count++;
            int pointDistance = points[i].distance;

            // Check for minimum distance
            if (pointDistance < clusterData[clusterId].minDistance && pointDistance != 0)
            {
                clusterData[clusterId].minDistance = pointDistance;
                clusterData[clusterId].minDistancePointIndex = i;
            }
            // Check for core point (based on number of neighbors)
            std::vector<int> neighbors;
            getNeighbors(points, i, neighbors);
            int numNeighbors_core = neighbors.size();
            if (numNeighbors_core >= minPoints)
            {
                int currentDistance = points[i].distance;
                // Validate the current point's distance
                if (currentDistance > 0 && currentDistance < Limit_distance)
                {
                    // Assuming the core point is the one with the most neighbors
                    if (clusterData[clusterId].corePointIndex == -1 || numNeighbors_core > clusterData[clusterId].corePointNumNeighbors)
                    {
                        clusterData[clusterId].corePointIndex = i;
                        clusterData[clusterId].corePointNumNeighbors = numNeighbors_core;

                        int corePointIndex = clusterData[clusterId].corePointIndex;
                        if (corePointIndex >= 0 && corePointIndex < numPoints)
                        {

                            clusterData[clusterId].corePointDistance = points[corePointIndex].distance; // Access the distance using flatIndex
                        }
                    }
                }
            }
        }
    }

    // Populate the clusters vector
    for (int i = 0; i <= numberOfClusters; ++i)
    {
        if (clusterData[i].count > 0)
        {
            clusterData[i].id = i;
            clusterData[i].centroidX = clusterData[i].sumX / clusterData[i].count;
            clusterData[i].centroidY = clusterData[i].sumY / clusterData[i].count;
            clusterData[i].centroidZ = clusterData[i].sumZ / clusterData[i].count;
            clusterData[i].centerDistance = sqrt(pow(clusterData[i].centroidX, 2) + pow(clusterData[i].centroidY, 2) + pow(clusterData[i].centroidZ, 2));
            clusters.push_back(clusterData[i]);
        }
    }
    return clusters;
}

void printClustersInfo(const std::vector<ClusterInfo> clusters, int sweep)
{
    Serial.print("\t Formed Clusters\t \n");
    for (const auto &cluster : clusters)
    {
        Serial.print("Cluster Centroid: (");
        Serial.print(cluster.centroidX);
        Serial.print(", ");
        Serial.print(cluster.centroidY);
        Serial.print(", ");
        Serial.print(cluster.centroidZ);
        Serial.print(")");
        Serial.print(" , ");
        Serial.print("Distance: ");
        Serial.print(cluster.centerDistance);
        Serial.print(" , ");
        Serial.print("Number of Points: ");
        Serial.print(cluster.count);
        Serial.print(" , Core Point Num Neighbors: ");
        Serial.print(cluster.corePointNumNeighbors);
        Serial.print(" , Core Point distance: ");
        Serial.print(cluster.corePointDistance);
        Serial.print(" , Minimum Distance: ");
        Serial.println(cluster.minDistance);
    }
}

void printPadded(String value, int width)
{
    int numSpaces = width - value.length();
    Serial.print(value);
    for (int i = 0; i < numSpaces; i++)
    {
        Serial.print(" ");
    }
    Serial.print(" | ");
}

void print_clusters_points(Point points[], std::map<int, std::vector<Point>> clusterMap)
{
    Serial.println(" ID | Coordinates (x, y, z)      | Distance  |  Pan  | Tilt ");
    Serial.println("----|----------------------------|-----------|-------|------");

    for (const auto &pair : clusterMap)
    {
        const auto &points = pair.second;

        for (const auto &point : points)
        {
            printPadded(String(pair.first), 3); // Adjust the width as needed

            String coordinates = "(" + String(point.x, 2) + ", " + String(point.y, 2) + ", " + String(point.z, 2) + ")";
            printPadded(coordinates, 26); // Width based on your data

            printPadded(String(point.distance), 9);
            printPadded(String(point.pan_angle, 1), 5);
            printPadded(String(point.tilt_angle, 1), 5);

            Serial.println();
        }
        Serial.println("-------------------------------------------------------------");
    }
}