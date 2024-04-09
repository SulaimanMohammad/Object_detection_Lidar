#ifndef CLUSTERINFO_H
#define CLUSTERINFO_H
#include "Characterization.h"
#include "Coordination.h"
#include "DBSCAN.h"

struct ClusterInfo
{
    float sumX, sumY, sumZ;                // Coordinates for centroid
    float centroidX, centroidY, centroidZ; // Centroid coordinates
    float centerDistance;                  // DIstance of center of cluster from sensor
    int count;                             // Count of points in the cluster
    int corePointIndex;                    // Index of the core point
    float corePointDistance;               // Distance of the core point
    float minDistance;                     // Minimum distance in the cluster
    int minDistancePointIndex;             // Index of the point with minimum distance
    int id;                                // id of cluster
    float topsisScore;                     // scores based on criteria
    int sweep1Index;                       // Index of the cluster in clustersSweep1
    int sweep2Index;                       // Index of the cluster in clustersSweep2
};

// TODO gather_clusters_info uses 2 vectors it is enough to use only one then remove the ones with count ZEROS
std::vector<ClusterInfo> gather_clusters_info(Point points[], int numberOfClusters); // Clacluate the center of each cluster and find aslo number of points, core point ....
void printClustersInfo(const std::vector<ClusterInfo> clusters, int sweep);
void print_clusters_points(Point points[], std::map<int, std::vector<Point>> clusterMap);
#endif // CLUSTER_INFO_H
