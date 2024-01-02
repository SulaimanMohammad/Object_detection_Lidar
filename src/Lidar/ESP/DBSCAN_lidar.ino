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

// Point that realtes between disstance and angle in 2D plan
struct Point
{
    float x, y;
    double angle;
};
int savedDistances[(full_FOV / FOV) + 1]; // Array to store distances at each FOV step
// +1 because 18/18=10 (0 to 162) and we need one element for the read of 180
bool isFirstReading[full_FOV / FOV + 1]; // Array to track if it's the first reading for each position

// initialization .
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, 2);
Servo myservo;

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