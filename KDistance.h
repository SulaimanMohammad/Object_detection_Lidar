#ifndef KDISTANCE_H
#define KDISTANCE_H
#include "Coordination.h"

void calculateDimensionKDistance(Point points[], std::vector<float> &kDistances, int dimension);        // Claculate K-distance for each Dimension
std::vector<std::vector<float>> calculateKDistance_set_Epsilon(Point points[], float &epsilonX, float &epsilonY, float &epsilonZ);
int findKneePointIndex(const std::vector<float> &kDistances);
void print_kDistance(std::vector<float> kDistances);

#endif
