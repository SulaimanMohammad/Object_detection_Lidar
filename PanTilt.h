#ifndef PANTILT_H
#define PANTILT_H

#include <ESP32Servo.h>
#include "Coordination.h"
#include "Characterization.h"
#include "MeasureDistance.h"

extern Servo pan_servo;
extern Servo tilt_servo;
void configure_servos();
void move_servo_to_position(Servo &servo, int position);                                                     // Physically move the servos to target angle
Point update_point(Point points[], int pan_servoPosition, int tilt_servoPosition, int distance, int index);  // mapp the point based on the read distance
int calculate_index(int pan_servoPosition, int tilt_servoPosition);                                          // find the index of point based on the pan and tilt angle (unique access)
void calculate_tilt_range(int &tiltStart, int &tiltEnd, int &tiltStep, int pan_servoPosition, bool reverse); // find where is the next angle of tilt to scan zigzag
void print_point_data(Point points[], int index, int pan_servoPosition, int tilt_servoPosition, int distance);
void move_collect_data(Point points[], Servo &pan_servo, Servo &tilt_servo, int start_point, int end_point, bool print_data = false); // main function to move and store data

#endif
