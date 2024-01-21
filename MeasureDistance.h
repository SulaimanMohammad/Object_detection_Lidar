#ifndef MEASUREDISTANCE_H
#define MEASUREDISTANCE_H
#include "Characterization.h"
#include "ReadTOF.h"

int calculateMedianDistance(int samples[]); // Choose the median in case of multiple samples
void sortArray(int arr[], int numElements); // Needed for median
int get_distance();                         // Read the sensor data
#endif                                      // MEASUREDISTANCE_H