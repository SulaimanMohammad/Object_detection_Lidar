#ifndef DBSCAN_H
#define DBSCAN_H
#include "Coordination.h"
#include "KDistance.h"

std::map<int, std::vector<Point>> detect_objects_clustering(Point points[]);
int get_number_of_clusters(std::map<int, std::vector<Point>> clusterMap);                       // Return number of formed clusters
void DBSCAN(Point points[], int &clusterId);                                                    // Perform DBSCAN
void getNeighbors(Point points[], int pointIndex, std::vector<int> &neighbors);                 // FInd if the point is neighbor or not
bool arePointsNeighbors(const Point &p1, const Point &p2);                                      // Check difference between X,y,z of two points if they are in the limit of epsilons
void expandCluster(Point points[], int pointIndex, int clusterId, std::vector<int> &neighbors); // Extend Cluster when new point found that belongs

#endif // DBSCAN_H

