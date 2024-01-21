
#ifndef COORDINATION_H
#define COORDINATION_H

#include <Arduino.h>
#include "Characterization.h"
#include <string.h>
#include <vector>
#include <map>
/*--- Point represent coordinates of point that mapped from polar system to 3D cartesian system --*/
struct Point
{
    float x, y, z;
    int distance = Limit_distance; // initialized as the max distance that can the sensor return
    float pan_angle, tilt_angle;
    // For DBSCAN to recognize if the point is counted and if it belongs to a cluster or no
    bool visited = false;
    int clusterId = UNCLASSIFIED;
};
void resetData(Point points[]);                                                      // Reset data after sweeps 0-180 to 180-0
void print_position_distance(int arrayIndex, int servoPosition, int distance_value); // print data of each angle and distance
void reorderPointsForConsistentProcessing(Point points[]);                           // Reorder Point to match 0-180 with 180 to 0
Point sphericalToCartesian(int pan_angle, int tilt_angle, float distance);           // Convert point from ( angle, distance) to (x,y)
float calculateDistance(const Point &p1, const Point &p2);                           // Calculate spatial distance

#endif // COORDINATION_H