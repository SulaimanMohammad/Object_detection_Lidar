#include "Coordination.h"
/*
------------------------------------------------------------
------------------Set the point in 2D plan -----------------
------------------------------------------------------------
*/
int map_angle_centered_fixed_system(int angle, int limit)
{
    if (angle > 180)
    {
        angle = angle - 180; // map the angle fist to range (0-90) or Axes will flip
    }
    int mapped_angle = angle - (limit / 2.0);
    return mapped_angle;
}

Point sphericalToCartesian(int pan_angle, int tilt_angle, float distance)
{
    Point p;
    p.distance = distance;
    p.pan_angle = pan_angle;
    p.tilt_angle = tilt_angle;
    // Convert angles from degrees to radians
    float theta = static_cast<float>(map_angle_centered_fixed_system(p.pan_angle, pan_end_range)) * PI / 180.0f; // Pan angle in radians
    float phi = static_cast<float>(map_angle_centered_fixed_system(p.tilt_angle, tilt_end_range)) * PI / 180.0f; // Tilt angle in radians

    // Convert to Cartesian coordinates
    p.x = distance;
    p.y = distance * sin(theta) * cos(phi);
    p.z = distance * sin(phi);

    return p;
}

// Function to calculate Euclidean distance between two points with angle consideration
float calculateDistance(const Point &p1, const Point &p2)
{
    // Calculate the Euclidean distance in 3D
    float spatialDist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
    return spatialDist;
}

void resetData(Point points[])
{

    // Reset the points array
    for (int i = 0; i < MAX_POINTS; i++)
    {
        points[i] = Point(); // Reset each point (assuming default constructor sets it to a default state)
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

void reorderPointsForConsistentProcessing(Point points[])
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
