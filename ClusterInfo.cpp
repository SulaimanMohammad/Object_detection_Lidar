#include "ClusterInfo.h"
#include <cfloat> // Include for FLT_MAX
/*
----------------------------------------------------------------
---------------------- Info of clusters ------------------------
----------------------------------------------------------------
*/
std::vector<ClusterInfo> gather_clusters_info(Point points[], int numberOfClusters)
{
    std::vector<ClusterInfo> clusters;
    std::vector<ClusterInfo> clusterData(numberOfClusters + 1);

    // Initialize cluster data
    for (int i = 0; i <= numberOfClusters; ++i)
    {
        clusterData[i] = {0, 0, 0,
                          0, 0, 0,
                          0,
                          0,
                          -1, 0,
                          Limit_distance, -1, 0, 0, 0, 0}; // Initialize minDistance with FLT_MAX
    }

    // Iterating through each point to accumulate data for centroid calculation and finding the minDistance
    for (int i = 0; i < numPoints; ++i)
    {
        int clusterId = points[i].clusterId;
        if (clusterId > 0 && clusterId <= numberOfClusters)
        {
            clusterData[clusterId].sumX += points[i].x;
            clusterData[clusterId].sumY += points[i].y;
            clusterData[clusterId].sumZ += points[i].z;
            clusterData[clusterId].count++;
            int pointDistance = points[i].distance;
            // Check for minimum distance
            if (pointDistance < clusterData[clusterId].minDistance && pointDistance > 0)
            {
                clusterData[clusterId].minDistance = pointDistance;
                clusterData[clusterId].minDistancePointIndex = i;
            }
        }
    }

    // Calculating centroids and identifying core point based on closest distance to centroid
    for (int clusterId = 1; clusterId <= numberOfClusters; ++clusterId)
    {
        if (clusterData[clusterId].count > 0)
        {
            clusterData[clusterId].id = clusterId;
            clusterData[clusterId].centroidX = clusterData[clusterId].sumX / clusterData[clusterId].count;
            clusterData[clusterId].centroidY = clusterData[clusterId].sumY / clusterData[clusterId].count;
            clusterData[clusterId].centroidZ = clusterData[clusterId].sumZ / clusterData[clusterId].count;
            clusterData[clusterId].centerDistance = sqrt(pow(clusterData[clusterId].centroidX, 2) + pow(clusterData[clusterId].centroidY, 2) + pow(clusterData[clusterId].centroidZ, 2));

            float minDistToCentroid = FLT_MAX;
            for (int i = 0; i < numPoints; ++i)
            {
                if (points[i].clusterId == clusterId)
                {
                    float distToCentroid = sqrt(pow(points[i].x - clusterData[clusterId].centroidX, 2) +
                                                pow(points[i].y - clusterData[clusterId].centroidY, 2) +
                                                pow(points[i].z - clusterData[clusterId].centroidZ, 2));
                    if (distToCentroid < minDistToCentroid && distToCentroid < Limit_distance)
                    {
                        minDistToCentroid = distToCentroid;
                        clusterData[clusterId].corePointIndex = i;
                        clusterData[clusterId].corePointDistance = points[i].distance;
                    }
                }
            }
            clusters.push_back(clusterData[clusterId]);
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
        Serial.print("num of points");
        Serial.println(pair.second.size());
        for (const auto &point : points)
        {
            printPadded(String(pair.first), 3);

            String coordinates = "(" + String(point.x, 2) + ", " + String(point.y, 2) + ", " + String(point.z, 2) + ")";
            printPadded(coordinates, 26);

            printPadded(String(point.distance), 9);
            printPadded(String(point.pan_angle, 1), 5);
            printPadded(String(point.tilt_angle, 1), 5);

            Serial.println();
        }
        Serial.println("-------------------------------------------------------------");
    }
}
