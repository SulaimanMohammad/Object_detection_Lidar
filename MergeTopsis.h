#ifndef MERGETOPSIS_H
#define MERGETOPSIS_H
#include "ClusterInfo.h"
#include <cfloat>

void normalizeClustersData(std::vector<ClusterInfo> &clusters);                       // used to calculate Scores
void Reverse_normalization(ClusterInfo &cluster);                                     // Reverse to preserve the info of the cluster staying same
void calculateTopsisScores(std::vector<ClusterInfo> &clusters);                       // Scores that are given to each cluster based on criteria of the cluster to decide the merge process
ClusterInfo mergeTwoClusters(const ClusterInfo &c1, const ClusterInfo &c2);           // Merge clusters to make them one and set the info of the result cluster
std::vector<ClusterInfo> mergeClustersPostTopsis(std::vector<ClusterInfo> &clusters); // some times the result of first merge is not enough and final merge needed
std::vector<ClusterInfo> mergeClustersBasedOnTopsis(std::vector<ClusterInfo> &clustersSweep1, std::vector<ClusterInfo> &clustersSweep2);

#endif