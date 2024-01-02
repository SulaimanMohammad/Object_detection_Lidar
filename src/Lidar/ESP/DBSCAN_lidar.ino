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
#include <cfloat> // or <float.h> in C

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN
// parametrs for DBSCAN
#define NOISE -2
#define UNCLASSIFIED -1

// Lidar variables
const int numSamples = 2; // number of samples for one reading
#define FOV 2             // Field of View of the sensor
#define full_FOV 180
#define MAX_POINTS full_FOV / FOV // Adjust based on your maximum expected number of points

// Servo variables
int controle = 2;
int feedbackPin = 4;
int readingsCount = 0;
int servoPosition = 0; // Current position of the servo
int feedbackValue;

// Point represent connection between distance-angle in 2D plan
struct Point
{
    float x, y, distance;
    double angle;
    bool visited = false;
    int clusterId = UNCLASSIFIED;
};
int savedDistances[(full_FOV / FOV) + 1]; // Array to store distances at each FOV step
// +1 because 18/18=10 (0 to 162) and we need one element for the read of 180
bool isFirstReading[full_FOV / FOV + 1]; // Array to track if it's the first reading for each position

// DBSCAN parameters
float epsilon;
const int minPoints = 2;
const int MAX_CLUSTERS = MAX_POINTS;
int numPoints = 0;
Point points[MAX_POINTS];

// K-distance to define espsilon of DBSCAN
struct KDistance
{
    float distance;
    int index;
};
const int K = minPoints;
float coeff_elbow = 0.7; // 70%
float coeff_Knee = 1 - coeff_elbow;

struct ClusterCenter
{
    float x, y;
};
ClusterCenter clusterSums[MAX_CLUSTERS];
int clusterCounts[MAX_CLUSTERS];

// Initialization .
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, 2);
Servo myservo;

// Declear functions
Point polarToCartesian(int servo_angle, float distance);
float calculateDistance(const Point &p1, const Point &p2);
void DBSCAN();
void expandCluster(int pointIndex, int clusterId, int *neighbors, int &numNeighbors);
void getNeighbors(int pointIndex, int *neighbors, int &numNeighbors);
void initializeClusterData();
float findKneePoint(const std::vector<float> &kDistances);
float findElbowPoint(float kDistances[], int numPoints);
void calculateKDistance(Point points[], int numPoints, float kDistances[]);
void calculateCentroids();
int get_distance();
int sensor_measurement();
int calculateMedianDistance(int samples[], int numSamples);
int updateAverage(int currentAverage, int newDistance, bool isFirstReading);
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
    // first reading passage
    for (int i = 0; i < sizeof(isFirstReading) / sizeof(isFirstReading[0]); i++)
    {
        isFirstReading[i] = true;
    }
}

void loop()
{
    // Reset numPoints before collecting new data
    numPoints = 0;
    // Move from 0° to 180°, updating the average and collecting valid data
    for (servoPosition = 0; servoPosition <= full_FOV; servoPosition += FOV)
    {
        myservo.write(servoPosition);
        delay(50); // Wait for stabilization
        feedbackValue = 0;
        int distance = get_distance();
        if (distance > 0 && numPoints < MAX_POINTS)
        { // Check if the reading is valid
            int arrayIndex = servoPosition / FOV;
            savedDistances[arrayIndex] = updateAverage(savedDistances[arrayIndex], distance, isFirstReading[arrayIndex]);
            isFirstReading[arrayIndex] = false;

            // Convert valid polar coordinates to Cartesian and store in the array
            points[numPoints] = polarToCartesian(servoPosition, savedDistances[arrayIndex]);
            SerialPort.print("Position: ");
            SerialPort.print(servoPosition);
            SerialPort.print(", Updated Average: ");
            SerialPort.println(savedDistances[arrayIndex]);
            numPoints++; // Increment the number of points
        }
    }

    // Calculate k-distance for each point
    float kDistances[MAX_POINTS];
    calculateKDistance(points, numPoints, kDistances);

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

    // Calculate espilon
    epsilon = coeff_elbow * findElbowPoint(kDistances, numPoints) + coeff_Knee * findKneePoint(kDistances);

    if (numPoints > 0)
    {
        // Apply DBSCAN for clustering only on valid points
        initializeClusterData();
        DBSCAN();
        calculateCentroids();
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

        Serial.print("Number of objects detected: ");
        Serial.println(numUniqueClusters);
    }
    else
    {
        Serial.println("No valid data points collected.");
    }

    myservo.write(0);
    delay(2000); // Wait for stabilization
    Serial.println("-------------------------------------------------");
}

/*
--------------------------------- Set the point in 2D plan -----------------------
*/
Point polarToCartesian(int servo_angle, float distance)
{
    Point p;
    double radian = static_cast<float>(servo_angle) * PI / 180.0f;
    p.angle = radian;
    p.x = distance * cos(radian);
    p.y = distance * sin(radian);
    return p;
}

// Function to calculate Euclidean distance between two points
float calculateDistance(const Point &p1, const Point &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

/*--------------------------------- DBSCAN -----------------------*/
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

// Function to initialize cluster arrays
void initializeClusterData()
{
    for (int i = 0; i < MAX_CLUSTERS; ++i)
    {
        clusterSums[i] = {0, 0};
        clusterCounts[i] = 0;
    }
}

/*
--------------------------------- K-Distance to define epsilon od DBSCAN -----------------------
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

/*
--------------------------------- Find epsilon od DBSCAN -----------------------
*/
float calculatePointDistance(const Point &p1, const Point &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}
// Elbow Method
float findElbowPoint(float kDistances[], int numPoints)
{
    float maxDifference = 0;
    int elbowPoint = -1;

    for (int i = 1; i < numPoints; ++i)
    {
        float difference = abs(kDistances[i] - kDistances[i - 1]);
        if (difference > maxDifference)
        {
            maxDifference = difference;
            elbowPoint = i;
        }
    }

    if (elbowPoint != -1)
    {
        Serial.print("Elbow Point at Index: ");
        Serial.println(elbowPoint);
        Serial.print("Suggested Epsilon: ");
        Serial.println(kDistances[elbowPoint]);
        return kDistances[elbowPoint];
    }
}

// Knee method ( consider many Elbows)
float findKneePoint(const std::vector<float> &kDistances)
{
    int numPoints = kDistances.size();
    if (numPoints == 0)
        return -1;

    // Explicitly create Point objects
    Point start;
    start.x = 0;
    start.y = kDistances[0];

    Point end;
    end.x = static_cast<float>(numPoints - 1);
    end.y = kDistances.back();

    float maxDistance = -1;
    int kneeIndex = -1;

    for (int i = 0; i < numPoints; ++i)
    {
        Point current;
        current.x = static_cast<float>(i);
        current.y = kDistances[i];

        float numerator = fabs((end.y - start.y) * current.x - (end.x - start.x) * current.y + end.x * start.y - end.y * start.x);
        float denominator = calculatePointDistance(start, end);

        float distance = numerator / denominator;
        if (distance > maxDistance)
        {
            maxDistance = distance;
            kneeIndex = i;
        }
    }

    if (kneePoint != -1)
    {
        Serial.print("Knee Point at Index: ");
        Serial.println(kneePoint);
        Serial.print("Suggested Epsilon: ");
        Serial.println(kDistances[kneePoint]);
        return kDistances[kneePoint];
    }
}

/*
--------------------------------- Info of clusters -----------------------
*/

void calculateCentroids()
{
    struct ClusterInfo
    {
        float sumX, sumY;
        int count;
        int corePointIndex;
        int corePointNumNeighbors; // Number of neighbors for the core point
        float minDistance;
        int minDistancePointIndex;
    };

    ClusterInfo clusterData[MAX_CLUSTERS];

    // Initialize cluster data
    for (int i = 0; i < MAX_CLUSTERS; ++i)
    {
        clusterData[i] = {0, 0, 0, -1, 0, FLT_MAX, -1};
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
            if (savedDistances[i] < clusterData[clusterId].minDistance)
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
                }
            }
        }
    }

    // Reporting the centroids, core points, and min distance points
    for (int i = 1; i < MAX_CLUSTERS; ++i)
    {
        if (clusterData[i].count > 0)
        {
            ClusterCenter centroid;
            centroid.x = clusterData[i].sumX / clusterData[i].count;
            centroid.y = clusterData[i].sumY / clusterData[i].count;

            Serial.print("Cluster ");
            Serial.print(i);
            Serial.print(" Centroid: (");
            Serial.print(centroid.x);
            Serial.print(", ");
            Serial.print(centroid.y);
            Serial.println(")");

            Serial.print("Core Point Index: ");
            Serial.println(clusterData[i].corePointIndex);
            Serial.print("Min Distance Point Index: ");
            Serial.println(clusterData[i].minDistancePointIndex);
            Serial.print("Min Distance: ");
            Serial.println(clusterData[i].minDistance);
        }
    }
}

/*
--------------------------------- Read data from sensor -----------------------
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
            // SerialPort.println("No valid object detected");
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

int updateAverage(int currentAverage, int newDistance, bool isFirstReading)
{
    if (isFirstReading)
    {
        // If it's the first reading, just return the new distance
        return newDistance;
    }
    else
    {
        // Otherwise, calculate the new average
        return (currentAverage + newDistance) / 2;
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

/*--------------ESP info-performance-----------*/
void reportMemory()
{
    // Report Free Heap Memory
    Serial.print("Free Heap Memory: ");
    Serial.println(ESP.getFreeHeap());

    // Optionally, report other memory-related information
    // Example: Maximum block of heap that can be allocated at once
    Serial.print("Largest Free Block: ");
    Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // Add more memory reporting as needed
}