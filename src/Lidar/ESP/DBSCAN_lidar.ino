#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <ESP32Servo.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <map>
#include <numeric>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

/* ---------- Lidar variables ---------- */
const int numSamples = 3; // number of samples for one reading
#define FOV 2             // Field of View of the sensor
#define full_FOV 180
#define MAX_POINTS (full_FOV / FOV) + 1 // Adjust based on maximum expected number of points
#define Sensor_range 6000               // it is maximum reading the sensor give here 6m in mm
#define Limit_distance Sensor_range + (Sensor_range * 0.20)
/* ---------- Servo variables ---------- */
int controle = 2;    // Pin on the bord for control signal
int feedbackPin = 4; // Pin on the bord for the feedback if it is used
int readingsCount = 0;
int servoPosition = 0; // Current position of the servo

/* ---------- DBSCAN parameters ---------- */
const int minPoints = 4;             // Number of close point to form cluster
float angleWeight = 0.4;             // How imortant is the connectivity of the angle with distance
float shift_margin_to_merge = 100;   // Max diffeence in distance between tow clusters core to merge them
float epsilon;                       // Dynamically defined by K-distance methode
const int MAX_CLUSTERS = MAX_POINTS; // Maximum point of cluster
int numPoints = 0;
#define NOISE -2
#define UNCLASSIFIED -1
struct ClusterInfo
{
    float sumX, sumY;           // Coordinates for centroid
    float centroidX, centroidY; // Centroid coordinates
    int count;                  // Count of points in the cluster
    int corePointIndex;         // Index of the core point
    float corePointDistance;    // Distance of the core point
    int corePointNumNeighbors;  // Number of neighbors for the core point
    float minDistance;          // Minimum distance in the cluster
    int minDistancePointIndex;  // Index of the point with minimum distance
    int sweep;                  // 1 for first sweep (0-180), 2 for second sweep (180-0)
    float topsisScore;
    int sweep1Index; // Index of the cluster in clustersSweep1
    int sweep2Index; // Index of the cluster in clustersSweep2
};

/* ---------- TOPSIS parameters ---------- */
float weightCorePointDistance = 0.5;
float weightNumberOfPoints = 0.01;
float weightMinDistance = 0.49;
// First merge between 2 sweeps
const float scoreMergeThreshold = 0.05; // Define a threshold for merging clusters based on TOPSIS score similarity
const float distanceMergeThreshold = 200;
// Second merge of the result of first one for general view of objects
const float PostMergeTOPSIS_scoreThreshold = 0.02;
const float PostMergeTOPSIS_distanceThreshold = 100;
float globalMaxCorePointDistance = 0;
float globalMaxNumberOfPoints = 0;
float globalMaxMinDistance = Limit_distance;

/* ----------  K-distance to define espsilon of DBSCAN parameters ---------- */
const int K = minPoints;
float coeff_elbow = 0;
float coeff_Knee = 1 - coeff_elbow;
struct KDistance
{
    float distance;
    int index;
};

/*--- Point represent connection between distance-angle in 2D plan --*/
struct Point
{
    float x, y, distance;
    double angle;
    bool visited = false;
    int clusterId = UNCLASSIFIED;
};
Point points[MAX_POINTS];
// int savedDistances[(full_FOV / FOV) + 1]; // Array to store distances at each FOV step
// // +1 because 18/18=10 (0 to 162) and we need one element for the read of 180
std::vector<int> savedDistances((full_FOV / FOV) + 1, Limit_distance); // Vector to store distances at each FOV step
std::vector<bool> isFirstReading((full_FOV / FOV) + 1, true);          // Vector to track if it's the first reading for each position

/* ----------  Initialization ---------- */
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, 2);
Servo myservo;

/*----------- Functions Declaration -------*/
void move_collect_data(int start_point, int end_point);                              // Move servo and save distance for each angle
void detect_objects_clustering(std::vector<ClusterInfo> &clusters);                  // Call DBSCAN, calcul esp and count number of objects
void resetData();                                                                    // Reset data after sweeps 0-180 to 180-0
void print_position_distance(int arrayIndex, int servoPosition, int distance_value); // print data of each angle and distance
void reorderPointsForConsistentProcessing();                                         // Reorder Point to match 0-180 with 180 to 0
Point polarToCartesian(int servo_angle, float distance);                             // Convert point from ( angle, distance) to (x,y)
float calculateDistance(const Point &p1, const Point &p2);                           // Calculate the distance between 2 points considering the distance and the angle
void DBSCAN();
void expandCluster(int pointIndex, int clusterId, int *neighbors, int &numNeighbors);
void getNeighbors(int pointIndex, int *neighbors, int &numNeighbors);
void normalizeClustersData(std::vector<ClusterInfo> &clusters);
void calculateTopsisScores(std::vector<ClusterInfo> &clusters);
ClusterInfo mergeTwoClusters(const ClusterInfo &c1, const ClusterInfo &c2);
std::vector<ClusterInfo> mergeClustersBasedOnTopsis(std::vector<ClusterInfo> &clustersSweep1, std::vector<ClusterInfo> &clustersSweep2);
std::vector<ClusterInfo> mergeClustersPostTopsis(std::vector<ClusterInfo> &clusters);
float findKneePoint(float kDistances[], int numPoints);
float findElbowPoint(float kDistances[], int numPoints);
void calculateKDistance(Point points[], int numPoints, float kDistances[]);
void print_kDistance(float kDistances[], int numPoints);
void gather_clusters_info(std::vector<ClusterInfo> &clusters);
void printClustersInfo(const std::vector<ClusterInfo> &clusters, int sweep);
int get_distance();
int sensor_measurement();
int calculateMedianDistance(int samples[], int numSamples);
void sortArray(int arr[], int numElements);
void reportMemory();

void setup()
{
    pinMode(LedPin, OUTPUT);

    SerialPort.begin(115200); // Initialize serial for output.
    SerialPort.println("Starting...");
    DEV_I2C.begin();                                 // Initialize I2C bus.
    sensor_vl53l4cx_sat.begin();                     // Configure VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_Off();              // Switch off VL53L4CX satellite component.
    sensor_vl53l4cx_sat.InitSensor(0x12);            // Initialize VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement(); // Start Measurements

    // Set servo
    myservo.setPeriodHertz(50);
    myservo.attach(controle); // Attach the servo on GPIO 4
    myservo.write(0);         // Set initial position of servo to 0 degrees
}

void loop()
{
    Serial.println(" Sweep 0-180");
    move_collect_data(0, 180);
    std::vector<ClusterInfo> clustersSweep1;
    detect_objects_clustering(clustersSweep1);
    printClustersInfo(clustersSweep1, 1); // After first sweep

    Serial.println(" Sweep 180-0");
    move_collect_data(180, 0);
    reorderPointsForConsistentProcessing(); // Reorder points for consistent processing

    std::vector<ClusterInfo> clustersSweep2;
    detect_objects_clustering(clustersSweep2);
    printClustersInfo(clustersSweep2, 2);

    std::vector<ClusterInfo> mergedClusters = mergeClustersBasedOnTopsis(clustersSweep1, clustersSweep2);
    Serial.println("            Merged Cluster Info: ");
    for (const auto &cluster : mergedClusters)
    {
        Serial.print("CorePointDistance: ");
        Serial.print(cluster.corePointDistance);
        Serial.print(" \tMin Distance: ");
        Serial.println(cluster.minDistance);
    }
    resetData();
    delay(500); // Wait for stabilization
    Serial.println("-------------------------------------------------");
    // All vectors sweep1,sweep2 and merged cluster are local of each iteration, at end of each iter they will be destroyed and mem free
}

/*
------------------------------------------------------------
--------------------- Main functions -----------------------
------------------------------------------------------------
 */
void move_collect_data(int start_point, int end_point)
{
    int step = (start_point < end_point) ? FOV : -FOV; // Determine the direction based on start and end values

    // Reset numPoints before collecting new data
    numPoints = 0;

    // Move from start to end, updating the average and collecting valid data
    for (int servoPosition = start_point; (step > 0) ? (servoPosition <= end_point) : (servoPosition >= end_point); servoPosition += step)
    {
        myservo.write(servoPosition);
        delay(50); // Wait for stabilization
        int distance = get_distance();
        if (distance > 0 && numPoints < MAX_POINTS) // Check if the reading is valid
        {
            int arrayIndex = abs(servoPosition) / FOV;
            if (isFirstReading[arrayIndex] || savedDistances[arrayIndex] == Limit_distance)
            {
                // If it's the first reading, just return the new distance
                savedDistances[arrayIndex] = distance;
            }
            else
            {
                // Otherwise, calculate the new average
                savedDistances[arrayIndex] = savedDistances[arrayIndex] + distance / 2.0;
            }
            isFirstReading[arrayIndex] = false;

            // Convert valid polar coordinates to Cartesian and store in the array
            points[numPoints] = polarToCartesian(servoPosition, savedDistances[arrayIndex]);
            print_position_distance(arrayIndex, servoPosition, savedDistances[arrayIndex]);

            numPoints++; // Increment the number of points
        }
        // Additional check to ensure the loop includes the end point
        if (servoPosition == end_point)
        {
            break;
        }
    }
}

void detect_objects_clustering(std::vector<ClusterInfo> &clusters)
{
    // Calculate k-distance for each point
    float kDistances[MAX_POINTS];
    calculateKDistance(points, numPoints, kDistances);
    // print_kDistance(kDistances, numPoints);
    // Calculate espilon
    int Elbow_value = findElbowPoint(kDistances, numPoints);
    int Knee_value = findKneePoint(kDistances, numPoints);
    epsilon = coeff_elbow * Elbow_value + coeff_Knee * Knee_value;
    Serial.print("Elbow_value= ");
    Serial.print(Elbow_value);
    Serial.print(" ,Knee_value= ");
    Serial.print(Knee_value);
    Serial.print(",Chosed Epsilon= ");
    Serial.println(epsilon);

    if (numPoints > 0)
    {
        // Apply DBSCAN for clustering only on valid points
        DBSCAN();
        // Count clusters (objects) in valid points
        int uniqueClusters[MAX_POINTS] = {0};
        int numUniqueClusters = 0;

        for (int i = 0; i < numPoints; i++)
        {
            if (points[i].clusterId > 0)
            {
                if (uniqueClusters[points[i].clusterId] == 0)
                {
                    uniqueClusters[points[i].clusterId] = 1; // Mark this cluster ID as found
                    numUniqueClusters++;
                }
            }
        }

        Serial.print("Number of objects detected:");
        Serial.println(numUniqueClusters);
    }
    else
    {
        Serial.println("No valid data points collected.");
    }
    gather_clusters_info(clusters); // Save the info of clusters in vector
}

void resetData()
{
    for (int i = 0; i < MAX_POINTS; i++)
    {
        savedDistances[i] = 0;
        isFirstReading[i] = true;
        // Reset the points array
        points[i] = Point();
        points[i].clusterId = UNCLASSIFIED;
        points[i].visited = false;
    }
}

void print_position_distance(int arrayIndex, int servoPosition, int distance_value)
{
    Serial.print("arrayIndex");
    Serial.print(arrayIndex);
    SerialPort.print(" ,Position: ");
    SerialPort.print(servoPosition);
    SerialPort.print(", Updated Average: ");
    SerialPort.println(distance_value);
}

void reorderPointsForConsistentProcessing()
{
    int i = 0;
    int j = numPoints - 1;
    while (i < j)
    {
        // Swap points[i] and points[j]
        Point temp = points[i];
        points[i] = points[j];
        points[j] = temp;
        i++;
        j--;
    }
}
/*
------------------------------------------------------------
------------------Set the point in 2D plan -----------------
------------------------------------------------------------
*/
Point polarToCartesian(int servo_angle, float distance)
{
    Point p;
    p.angle = servo_angle;
    double radian = static_cast<float>(servo_angle) * PI / 180.0f;
    p.x = distance * cos(radian);
    p.y = distance * sin(radian);
    return p;
}

// Function to calculate Euclidean distance between two points
float calculateDistance(const Point &p1, const Point &p2)
{
    float distanceDiff = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)); // Euclidean distance
    float angleDiff = fabs(p1.angle - p2.angle);                          // Absolute difference in angles
    return distanceDiff + angleWeight * angleDiff;                        // Combine using a weight factor
}

/*
-------------------------------------------------------------
--------------------------- DBSCAN --------------------------
-------------------------------------------------------------
*/
void DBSCAN()
{
    int clusterId = 0;
    for (int i = 0; i < numPoints; ++i)
    {
        if (points[i].clusterId == UNCLASSIFIED)
        {
            int neighbors[MAX_POINTS];
            int numNeighbors;

            getNeighbors(i, neighbors, numNeighbors);
            if (numNeighbors < minPoints)
            {
                points[i].clusterId = NOISE;
            }
            else
            {
                expandCluster(i, ++clusterId, neighbors, numNeighbors);
            }
        }
    }
}

void expandCluster(int pointIndex, int clusterId, int *neighbors, int &numNeighbors)
{
    points[pointIndex].clusterId = clusterId;

    for (int i = 0; i < numNeighbors; ++i)
    {
        int currNeighbor = neighbors[i];
        if (points[currNeighbor].clusterId == UNCLASSIFIED || points[currNeighbor].clusterId == NOISE)
        {
            points[currNeighbor].clusterId = clusterId;

            int newNeighbors[MAX_POINTS];
            int newNumNeighbors;
            getNeighbors(currNeighbor, newNeighbors, newNumNeighbors);

            for (int j = 0; j < newNumNeighbors; ++j)
            {
                // Check if new neighbor is already in neighbors array
                bool alreadyNeighbor = false;
                for (int k = 0; k < numNeighbors; ++k)
                {
                    if (neighbors[k] == newNeighbors[j])
                    {
                        alreadyNeighbor = true;
                        break;
                    }
                }
                if (!alreadyNeighbor && numNeighbors < MAX_POINTS)
                {
                    neighbors[numNeighbors++] = newNeighbors[j];
                }
            }
        }
    }
}

// Function to find neighbors of a point
void getNeighbors(int pointIndex, int *neighbors, int &numNeighbors)
{
    numNeighbors = 0;
    for (int i = 0; i < numPoints; ++i)
    {
        if (calculateDistance(points[pointIndex], points[i]) <= epsilon)
        {
            neighbors[numNeighbors++] = i;
        }
    }
}

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
        globalMaxMinDistance = std::min(globalMaxMinDistance, cluster.minDistance);
    }

    // Normalize the data
    for (auto &cluster : clusters)
    {
        cluster.corePointDistance /= globalMaxCorePointDistance;
        cluster.count = float(cluster.count) / globalMaxNumberOfPoints;
        cluster.minDistance /= globalMaxMinDistance;
    }
}

void Reverse_normalization(ClusterInfo &cluster)
{
    cluster.corePointDistance *= globalMaxCorePointDistance;
    cluster.count *= globalMaxNumberOfPoints;
    cluster.minDistance *= globalMaxMinDistance;
}

// Function to calculate TOPSIS score for each cluster
void calculateTopsisScores(std::vector<ClusterInfo> &clusters)
{

    // Normalize the cluster data
    normalizeClustersData(clusters);

    // Calculate the ideal and negative-ideal solutions
    float idealCorePointDistance = 1, idealNumberOfPoints = 1, idealMinDistance = 1;
    float negIdealCorePointDistance = 0, negIdealNumberOfPoints = 0, negIdealMinDistance = 0;

    // Calculate the separation measures and the relative closeness
    for (auto &cluster : clusters)
    {
        float distanceToIdeal = sqrt(pow(weightCorePointDistance * (idealCorePointDistance - cluster.corePointDistance), 2) +
                                     pow(weightNumberOfPoints * (idealNumberOfPoints - cluster.count), 2) +
                                     pow(weightMinDistance * (idealMinDistance - cluster.minDistance), 2));

        float distanceToNegIdeal = sqrt(pow(weightCorePointDistance * (cluster.corePointDistance - negIdealCorePointDistance), 2) +
                                        pow(weightNumberOfPoints * (cluster.count - negIdealNumberOfPoints), 2) +
                                        pow(weightMinDistance * (cluster.minDistance - negIdealMinDistance), 2));

        float denominator = distanceToNegIdeal + distanceToIdeal;
        if (denominator == 0)
        {
            Serial.println("Warning: Denominator is zero in TOPSIS score calculation");
        }

        cluster.topsisScore = distanceToNegIdeal / (distanceToNegIdeal + distanceToIdeal);
        Reverse_normalization(cluster);
    }

    // Sort the clusters based on their TOPSIS scores
    // std::sort(clusters.begin(), clusters.end(), [](const ClusterInfo &a, const ClusterInfo &b)
    //           { return a.topsisScore > b.topsisScore; });
}

ClusterInfo mergeTwoClusters(const ClusterInfo &c1, const ClusterInfo &c2)
{
    ClusterInfo mergedCluster;
    mergedCluster.sumX = (c1.sumX + c2.sumX) / 2.0;
    mergedCluster.sumY = (c1.sumY + c2.sumY) / 2.0;
    mergedCluster.count = (c1.count + c2.count) / 2.0;
    // mergedCluster.corePointDistance = (c1.corePointDistance + c2.corePointDistance) / 2;
    mergedCluster.corePointDistance = std::min(c1.corePointDistance, c2.corePointDistance);
    mergedCluster.minDistance = std::min(c1.minDistance, c2.minDistance);
    return mergedCluster;
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
/*
---------------------------------------------------------------
------------- K-Distance to define epsilon od DBSCAN ----------
---------------------------------------------------------------
*/
// Function to calculate the k-distance for each point
void calculateKDistance(Point points[], int numPoints, float kDistances[])
{
    for (int i = 0; i < numPoints; ++i)
    {
        std::vector<KDistance> distances;
        for (int j = 0; j < numPoints; ++j)
        {
            if (i != j)
            {
                float dist = calculateDistance(points[i], points[j]);
                distances.push_back({dist, j});
            }
        }
        std::sort(distances.begin(), distances.end(), [](const KDistance &a, const KDistance &b)
                  { return a.distance < b.distance; });
        if (distances.size() >= K)
        {
            kDistances[i] = distances[K - 1].distance;
        }
        else
        {
            kDistances[i] = -1; // Indicate insufficient neighbors
        }
    }
}

void print_kDistance(float kDistances[], int numPoints)
{
    for (int i = 0; i < numPoints; ++i)
    {
        if (kDistances[i] != -1)
        {
            Serial.print("Point ");
            Serial.print(i);
            Serial.print(" k-distance: ");
            Serial.println(kDistances[i]);
        }
    }
}
/*
---------------------------------------------------------------
---------------------- Find epsilon od DBSCAN -----------------
---------------------------------------------------------------
*/
float calculatePointDistance(const Point &p1, const Point &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// Elbow Method
float findElbowPoint(float kDistances[], int numPoints)
{
    std::vector<int> elbowIndices; // Store indices of all elbow points

    for (int i = 1; i < numPoints - 1; ++i)
    {
        // Finding local maxima in the k-distance graph
        if (kDistances[i] > kDistances[i - 1] && kDistances[i] > kDistances[i + 1])
        {
            elbowIndices.push_back(i);
        }
    }

    // Calculate the average of all elbow points
    float sumElbowDistances = 0;
    for (int index : elbowIndices)
    {
        sumElbowDistances += kDistances[index];
    }

    float averageElbowDistance = 0;
    if (!elbowIndices.empty())
    {
        averageElbowDistance = sumElbowDistances / elbowIndices.size();
    }

    return averageElbowDistance;
}

// Knee method ( consider many Elbows)
float findKneePoint_simple(float kDistances[], int numPoints)
{
    std::vector<int> kneeIndices; // Store indices of all knee points

    for (int i = 1; i < numPoints - 1; ++i)
    {
        // Finding points with significant change in slope
        float diff1 = kDistances[i] - kDistances[i - 1];
        float diff2 = kDistances[i + 1] - kDistances[i];
        if (diff1 * diff2 < 0)
        { // Change in the sign of the slope
            kneeIndices.push_back(i);
        }
    }

    // Calculate the average of all knee points
    float sumKneeDistances = 0;
    for (int index : kneeIndices)
    {
        sumKneeDistances += kDistances[index];
    }

    float averageKneeDistance = 0;
    if (!kneeIndices.empty())
    {
        averageKneeDistance = sumKneeDistances / kneeIndices.size();
    }

    return averageKneeDistance;
}

// this knee finder use Derivative-Based Method , Significant Knee Selection, Outlier Handling
float findKneePoint(float kDistances[], int numPoints)
{
    if (numPoints < 3)
    {
        return -1; // Not enough points to determine a knee
    }

    // Handling Outliers: Using percentile-based approach
    std::vector<float> sortedDistances(kDistances, kDistances + numPoints);
    std::sort(sortedDistances.begin(), sortedDistances.end());
    float lowerPercentile = sortedDistances[numPoints * 0.05]; // 5th percentile
    float upperPercentile = sortedDistances[numPoints * 0.95]; // 95th percentile

    // Finding points with significant change in slope (derivative-based)
    float maxSlopeChange = 0;
    int maxChangeIndex = -1;
    for (int i = 1; i < numPoints - 1; ++i)
    {
        if (kDistances[i] < lowerPercentile || kDistances[i] > upperPercentile)
        {
            continue; // Skip outliers
        }

        float diff1 = kDistances[i] - kDistances[i - 1];
        float diff2 = kDistances[i + 1] - kDistances[i];
        float slopeChange = std::abs(diff1 - diff2);

        if (diff1 * diff2 < 0 && slopeChange > maxSlopeChange)
        {
            maxSlopeChange = slopeChange;
            maxChangeIndex = i;
        }
    }

    // Return the knee point with the most significant change in slope
    return (maxChangeIndex != -1) ? kDistances[maxChangeIndex] : -1;
}

/*
----------------------------------------------------------------
---------------------- Info of clusters ------------------------
----------------------------------------------------------------
*/
void gather_clusters_info(std::vector<ClusterInfo> &clusters)
{
    ClusterInfo clusterData[MAX_CLUSTERS];

    // Initialize cluster data
    for (int i = 0; i < MAX_CLUSTERS; ++i)
    {
        clusterData[i] = {0, 0, 0, 0, 0, -1, 0, 0, Limit_distance, -1, 0}; // Initialize minDistance with FLT_MAX
    }

    // Iterating through each point
    for (int i = 0; i < numPoints; ++i)
    {
        int clusterId = points[i].clusterId;
        if (clusterId > 0 && clusterId < MAX_CLUSTERS)
        {
            // Accumulate for centroid calculation
            clusterData[clusterId].sumX += points[i].x;
            clusterData[clusterId].sumY += points[i].y;
            clusterData[clusterId].count++;
            // Check for minimum distance
            if (savedDistances[i] < clusterData[clusterId].minDistance && savedDistances[i] != 0)
            {

                clusterData[clusterId].minDistance = savedDistances[i];
                clusterData[clusterId].minDistancePointIndex = i;
            }

            // Check for core point (based on number of neighbors)
            int neighbors[MAX_POINTS];
            int numNeighbors = 0;
            getNeighbors(i, neighbors, numNeighbors);
            if (numNeighbors >= minPoints)
            {
                // Assuming the core point is the one with the most neighbors
                if (clusterData[clusterId].corePointIndex == -1 || numNeighbors > clusterData[clusterId].corePointNumNeighbors)
                {
                    clusterData[clusterId].corePointIndex = i;
                    clusterData[clusterId].corePointNumNeighbors = numNeighbors;

                    // Retrieve and store the distance of the core point
                    if (i < savedDistances.size())
                    {
                        clusterData[clusterId].corePointDistance = savedDistances[i];
                    }
                }
            }
        }
    }

    // Populate the clusters vector
    for (int i = 1; i < MAX_CLUSTERS; ++i)
    {
        if (clusterData[i].count > 0)
        {
            clusterData[i].centroidX = clusterData[i].sumX / clusterData[i].count;
            clusterData[i].centroidY = clusterData[i].sumY / clusterData[i].count;
            clusters.push_back(clusterData[i]);
        }
    }
}

void printClustersInfo(const std::vector<ClusterInfo> &clusters, int sweep)
{
    for (const auto &cluster : clusters)
    {
        // Serial.print("Cluster Centroid: (");
        // Serial.print(cluster.centroidX);
        // Serial.print(", ");
        // Serial.print(cluster.centroidY);
        // Serial.print(")");
        // Serial.print(" , ");
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

/*
-----------------------------------------------------------------
---------------------- Read data from sensor --------------------
-----------------------------------------------------------------
*/
int get_distance()
{
    int samples[numSamples];
    int validSamples = 0;
    long sumDistance = 0;
    int unvalid_counter = 0;
    for (int i = 0; i < numSamples; i++)
    {
        samples[i] = sensor_measurement();
        if (samples[i] < 5000 && samples[i] != -1)
        { // Assuming 8190 is the max value indicating an invalid reading
            sumDistance += samples[i];
            validSamples++;
        }
        else
        {
            unvalid_counter++;
        }
        delay(10);
    }

    if (unvalid_counter != numSamples)
    {
        int medianDistance = calculateMedianDistance(samples, numSamples);
        return medianDistance;
    }
    else
    {
        return -1;
    }
}

int sensor_measurement()
{
    VL53L4CX_MultiRangingData_t MultiRangingData;
    VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int status;
    int chosenDistance = INT_MAX;
    int chosenStatus = -1;
    int measurement = 0;
    do
    {
        status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0))
    {
        sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);

        for (int i = 0; i < pMultiRangingData->NumberOfObjectsFound; ++i)
        {
            int currentDistance = pMultiRangingData->RangeData[i].RangeMilliMeter;
            int currentStatus = pMultiRangingData->RangeData[i].RangeStatus;

            if ((currentStatus == 0 || currentStatus == 2 || currentStatus == 3 || currentStatus == 4 || currentStatus == 6 || currentStatus == 7) && currentDistance >= 0 && currentDistance < chosenDistance)
            {
                chosenDistance = currentDistance;
                chosenStatus = currentStatus;
            }
        }

        if (chosenStatus != -1)
        {
            measurement = chosenDistance;
        }
        else
        {
            measurement = -1;
        }

        if (status == 0)
        {
            sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
        }
    }
    return measurement;
}

// Function to calculate the median of an array of integers
int calculateMedianDistance(int samples[], int numSamples)
{
    // Sort the array of distances
    sortArray(samples, numSamples);

    // Compute the median
    if (numSamples % 2 != 0)
    { // Odd number of elements
        return samples[numSamples / 2];
    }
    else
    { // Even number of elements
        return (samples[numSamples / 2 - 1] + samples[numSamples / 2]) / 2;
    }
}

// Function to sort an array of integers
void sortArray(int arr[], int numElements)
{
    for (int i = 0; i < numElements - 1; i++)
    {
        for (int j = 0; j < numElements - i - 1; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                // Swap arr[j] and arr[j+1]
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
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