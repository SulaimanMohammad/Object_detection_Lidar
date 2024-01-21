#ifndef KDISTANCE_H
#define KDISTANCE_H
#include "Coordination.h"

void calculateDimensionKDistance(Point points[], std::vector<float> &kDistances, int dimension);        // Claculate K-distance for each Dimension
void calculateKDistance_set_Epsilon(Point points[], float &epsilonX, float &epsilonY, float &epsilonZ); // Choose espsilons
float findElbowPoint(const std::vector<float> &kDistances);                                             // Elbow methode to find espsilons, taking average of multiple elbows
float findKneePoint_simple(const std::vector<float> &kDistances);                                       // simple not enough accurate Knee methode
float findKneePoint(const std::vector<float> &kDistances);                                              // Angle-Based Knee Point Detection and it is the most accurate
// Adjust Thresholds Based On Feedback by estimating the number of noise points to choose the value of threshould ( depends on application)
int estimateNoisePoints(Point points[], float thresholdX, float thresholdY, float thresholdZ);
void adjustThresholdsBasedOnFeedback(Point points[], std::vector<float> kDistancesX, std::vector<float> kDistancesY, std::vector<float> kDistancesZ, float &epsilonX, float &epsilonY, float &epsilonZ);
void print_kDistance(std::vector<float> kDistances);

#endif