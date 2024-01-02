#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

// Sensor data .
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, 2);

void setup()
{
    // Led.
    pinMode(LedPin, OUTPUT);

    // Initialize serial for output.
    SerialPort.begin(115200);
    SerialPort.println("Starting...");

    // Initialize I2C bus.
    DEV_I2C.begin();

    // Configure VL53L4CX satellite component.
    sensor_vl53l4cx_sat.begin();

    // Switch off VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_Off();

    // Initialize VL53L4CX satellite component.
    sensor_vl53l4cx_sat.InitSensor(0x12);

    // Start Measurements
    sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
}

void loop()
{
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