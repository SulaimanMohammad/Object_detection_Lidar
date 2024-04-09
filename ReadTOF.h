#ifndef READTOF_H
#define READTOF_H
#include <Arduino.h>
#include "Characterization.h"

void configure_TOF();                // Configure the sensor
int sensor_measurement();            // read data and return -1 if the data is not available
#endif
