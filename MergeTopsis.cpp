#include "MergeTopsis.h"
float globalMaxCorePointDistance = FLT_MIN;
float globalMaxNumberOfPoints = FLT_MIN;
float globalMaxCenterDistance = FLT_MIN;
float globalMaxMinDistance = FLT_MIN;
/*
-------------------------------------------------------------
--------------------- Merge clusters ------------------------
-------------------------------------------------------------
*/
// Function to normalize the cluster data
void normalizeClustersData(std::vector<ClusterInfo> &clusters)
{
    // Find the maximum values
    for (const auto &cluster : clusters)
    {
        globalMaxCorePointDistance = std::max(globalMaxCorePointDistance, cluster.corePointDistance);
        globalMaxNumberOfPoints = std::max(globalMaxNumberOfPoints, float(cluster.count));
        globalMaxMinDistance = std::max(globalMaxMinDistance, cluster.minDistance);
        globalMaxCenterDistance = std::max(globalMaxCenterDistance, cluster.centerDistance);
    }
    // Normalize the data
    for (auto &cluster : clusters)
    {
        cluster.corePointDistance /= globalMaxCorePointDistance;
        cluster.count = float(cluster.count) / globalMaxNumberOfPoints;
        cluster.minDistance /= globalMaxMinDistance;
        cluster.centerDistance /= globalMaxCenterDistance;
    }
}

void Reverse_normalization(ClusterInfo &cluster)
{
    cluster.corePointDistance *= globalMaxCorePointDistance;
    cluster.count *= globalMaxNumberOfPoints;
    cluster.minDistance *= globalMaxMinDistance;
    cluster.centerDistance *= globalMaxCenterDistance;
}

// Function to calculate TOPSIS score for each cluster
void calculateTopsisScores(std::vector<ClusterInfo> &clusters)
{

    // Normalize the cluster data
    normalizeClustersData(clusters);

    // Calculate the ideal and negative-ideal solutions
    float idealCorePointDistance = 1, idealNumberOfPoints = 1, idealMinDistance = 1;
    float negIdealCorePointDistance = 0, negIdealNumberOfPoints = 0, negIdealMinDistance = 0;
    float idealCenterDistance = 1;
    float negIdealCenterDistance = 0;
    // Calculate the separation measures and the relative closeness
    for (auto &cluster : clusters)
    {
        float distanceToIdeal = sqrt(
            pow(weightCorePointDistance * (idealCorePointDistance - cluster.corePointDistance), 2) +
            pow(weightNumberOfPoints * (idealNumberOfPoints - cluster.count), 2) +
            pow(weightMinDistance * (idealMinDistance - cluster.minDistance), 2) +
            pow(weightCenterDistance * (idealCenterDistance - cluster.centerDistance), 2) // Include centerDistance
        );

        float distanceToNegIdeal = sqrt(
            pow(weightCorePointDistance * (cluster.corePointDistance - negIdealCorePointDistance), 2) +
            pow(weightNumberOfPoints * (cluster.count - negIdealNumberOfPoints), 2) +
            pow(weightMinDistance * (cluster.minDistance - negIdealMinDistance), 2) +
            pow(weightCenterDistance * (cluster.centerDistance - negIdealCenterDistance), 2) // Include centerDistance
        );

        float denominator = distanceToNegIdeal + distanceToIdeal;
        if (denominator == 0)
        {
            Serial.println("Warning: Denominator is zero in TOPSIS score calculation");
        }

        cluster.topsisScore = distanceToNegIdeal / (distanceToNegIdeal + distanceToIdeal);
        Reverse_normalization(cluster);
    }
}

ClusterInfo mergeTwoClusters(const ClusterInfo &c1, const ClusterInfo &c2)
{
    ClusterInfo mergedCluster;
    mergedCluster.sumX = (c1.sumX + c2.sumX) / 2.0;
    mergedCluster.sumY = (c1.sumY + c2.sumY) / 2.0;
    mergedCluster.count = (c1.count + c2.count) / 2.0;
    mergedCluster.corePointDistance = std::min(c1.corePointDistance, c2.corePointDistance);
    mergedCluster.minDistance = std::min(c1.minDistance, c2.minDistance);
    return mergedCluster;
}

std::vector<ClusterInfo> mergeClustersPostTopsis(std::vector<ClusterInfo> &clusters)
{
    if (clusters.size() <= 1)
    {
        return clusters; // No merging needed if there's one or no clusters
    }

    calculateTopsisScores(clusters); // Calculate TOPSIS scores for each cluster

    bool mergeHappened;

    do
    {
        mergeHappened = false;
        std::vector<ClusterInfo> newClusters;
        std::vector<bool> merged(clusters.size(), false);

        for (size_t i = 0; i < clusters.size(); ++i)
        {
            if (!merged[i])
            {
                ClusterInfo bestMergeCluster = clusters[i];
                int bestMergePartner = -1;
                float bestMergeScore = FLT_MAX;

                for (size_t j = i + 1; j < clusters.size(); ++j)
                {
                    if (!merged[j])
                    {
                        float scoreDiff = std::abs(clusters[i].topsisScore - clusters[j].topsisScore);
                        float distanceDiff = std::abs(clusters[i].minDistance - clusters[j].minDistance);

                        if (scoreDiff < PostMergeTOPSIS_scoreThreshold && distanceDiff < PostMergeTOPSIS_distanceThreshold && scoreDiff < bestMergeScore)
                        {
                            bestMergeScore = scoreDiff;
                            bestMergePartner = j;
                        }
                    }
                }

                if (bestMergePartner != -1)
                {
                    bestMergeCluster = mergeTwoClusters(bestMergeCluster, clusters[bestMergePartner]); // Assumes a function to merge two clusters
                    merged[bestMergePartner] = true;
                    mergeHappened = true;
                }

                newClusters.push_back(bestMergeCluster);
            }
        }

        clusters = newClusters;
    } while (mergeHappened);

    return clusters;
}

std::vector<ClusterInfo> mergeClustersBasedOnTopsis(std::vector<ClusterInfo> &clustersSweep1, std::vector<ClusterInfo> &clustersSweep2)
{
    // Calculate scores for all clusters
    calculateTopsisScores(clustersSweep1);
    calculateTopsisScores(clustersSweep2);

    std::vector<ClusterInfo> mergedClusters;
    std::vector<bool> mergedSweep1(clustersSweep1.size(), false);
    std::vector<bool> mergedSweep2(clustersSweep2.size(), false);

    for (size_t cluster1Index = 0; cluster1Index < clustersSweep1.size(); ++cluster1Index)
    {
        const auto &cluster1 = clustersSweep1[cluster1Index];
        float minDistanceDiff = Limit_distance;
        float closestScoreDiff = FLT_MAX;
        int closestClusterIndex = -1;

        // Find the closest cluster from sweep 2 based on TOPSIS score
        for (size_t i = 0; i < clustersSweep2.size(); ++i)
        {
            float scoreDiff = std::abs(cluster1.topsisScore - clustersSweep2[i].topsisScore);
            float distanceDiff = abs(cluster1.minDistance - clustersSweep2[i].minDistance);

            // Debugging: Print the TOPSIS score comparison
            /*
            Serial.print("Comparing Sweep 1 Cluster ");
            Serial.print(cluster1Index);
            Serial.print(" (Score: ");
            Serial.print(cluster1.topsisScore);
            Serial.print(") with Sweep 2 Cluster ");
            Serial.print(i);
            Serial.print(" (Score: ");
            Serial.print(clustersSweep2[i].topsisScore);
            Serial.print("), Score Difference: ");
            Serial.print(scoreDiff);
            Serial.print(" ,distanceDiff");
            Serial.println(distanceDiff);
            */

            if (distanceDiff < distanceMergeThreshold && scoreDiff < closestScoreDiff)
            {
                closestScoreDiff = scoreDiff;
                minDistanceDiff = distanceDiff;
                closestClusterIndex = i;
            }
        }

        // Merge the closest clusters if within the threshold
        if (closestClusterIndex != -1 && closestScoreDiff < scoreMergeThreshold)
        {
            ClusterInfo mergedCluster = mergeTwoClusters(cluster1, clustersSweep2[closestClusterIndex]);
            mergedCluster.sweep1Index = cluster1Index;
            mergedCluster.sweep2Index = closestClusterIndex;
            mergedClusters.push_back(mergedCluster);
            mergedSweep1[cluster1Index] = true;
            mergedSweep2[closestClusterIndex] = true;

            // Debugging: Print info about the merged clusters
            /*
            Serial.print("Merged Sweep 1 Cluster ");
            Serial.print(cluster1Index);
            Serial.print(" with Sweep 2 Cluster ");
            Serial.println(closestClusterIndex);
            */
        }
        else
        {
            ClusterInfo unmergedCluster = cluster1;
            unmergedCluster.sweep1Index = cluster1Index;
            unmergedCluster.sweep2Index = -1;
            mergedClusters.push_back(unmergedCluster);

            // Debugging: Print info about the unmerged cluster

            /*
            Serial.print("No suitable merge for Sweep 1 Cluster ");
            Serial.println(cluster1Index);
            */
        }
    }

    for (size_t i = 0; i < clustersSweep2.size(); ++i)
    {
        if (!mergedSweep2[i])
        {
            mergedClusters.push_back(clustersSweep2[i]);
        }
    }

    Serial.println(" First Merged Cluster Info: ");
    for (const auto &cluster : mergedClusters)
    {
        Serial.print("CorePointDistance: ");
        Serial.print(cluster.corePointDistance);
        Serial.print(" \tMin Distance: ");
        Serial.println(cluster.minDistance);
    }

    return mergeClustersPostTopsis(mergedClusters);
}