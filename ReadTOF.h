#ifndef READTOF_H
#define READTOF_H
#include <Wire.h>
#include <assert.h>
#include <vl53l4cx_class.h>
#define DEV_I2C Wire

extern VL53L4CX sensor_vl53l4cx_sat; // Instance of the sensor
void configure_TOF();                // Configure the sensor to operate using I2C
int sensor_measurement();            // read data and return -1 if the data is not available
#endif