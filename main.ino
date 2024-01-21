
#include <Arduino.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <numeric>

#include "Characterization.h"
#include "PanTilt.h"
#include "KDistance.h"
#include "DBSCAN.h"
#include "ClusterInfo.h"
#include "MergeTopsis.h"
#include "IdentifyObjects.h"

Point points[MAX_POINTS];

std::vector<ClusterInfo> performSweepAndProcess(int startRange, int endRange, int sweepNumber);
void mergeAndPrintClusterInfo(std::vector<ClusterInfo> &clustersSweep1, std::vector<ClusterInfo> &clustersSweep2);
void reportMemory();

void setup()
{
    SerialPort.begin(115200);
    configure_TOF();
    configure_servos();
}

void loop()
{

    auto clustersSweep1 = performSweepAndProcess(pan_start_range, pan_end_range, 1);
    auto clustersSweep2 = performSweepAndProcess(pan_end_range, pan_start_range, 2);

    mergeAndPrintClusterInfo(clustersSweep1, clustersSweep2);

    resetData(points);
    delay(100); // Wait for stabilization
    Serial.println("-------------------------------------------------");
}

std::vector<ClusterInfo> performSweepAndProcess(int startRange, int endRange, int sweepNumber)
{
    Serial.println(" Sweep " + String(startRange) + "-" + String(endRange));
    move_collect_data(points, pan_servo, tilt_servo, startRange, endRange);
    auto clusterMap = detect_objects_clustering(points);
    int numberOfClusters = get_number_of_clusters(clusterMap);
    auto clustersSweep = gather_clusters_info(points, numberOfClusters);
    printClustersInfo(clustersSweep, sweepNumber);
    print_clusters_points(points, clusterMap);
    auto detected_objects = define_objects(clustersSweep, clusterMap);
    printObjects(detected_objects);

    return clustersSweep; // Return the cluster data
}

void mergeAndPrintClusterInfo(std::vector<ClusterInfo> &clustersSweep1, std::vector<ClusterInfo> &clustersSweep2)
{
    auto mergedClusters = mergeClustersBasedOnTopsis(clustersSweep1, clustersSweep2);
    Serial.println("        Merged Cluster Info:           ");
    for (const auto &cluster : mergedClusters)
    {
        Serial.print("CorePointDistance: ");
        Serial.print(cluster.corePointDistance);
        Serial.print(" \tMin Distance: ");
        Serial.println(cluster.minDistance);
    }
}
/*
---------------------------------------------
--------------ESP info-performance-----------
---------------------------------------------
*/
void reportMemory()
{
    // Report Free Heap Memory
    Serial.print("Free Heap Memory: ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("Largest Free Block: ");
    Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}
