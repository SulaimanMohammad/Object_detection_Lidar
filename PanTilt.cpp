#include "PanTilt.h"
int points_in_warning_distance = 0;
Servo pan_servo;
Servo tilt_servo;
void configure_servos()
{
    // Set servo
    pan_servo.setPeriodHertz(50);
    tilt_servo.setPeriodHertz(50);
    pan_servo.attach(pan_controle);
    tilt_servo.attach(tilt_controle);
    pan_servo.write(0);
    tilt_servo.write(0);
}
/*
------------------------------------------------------------
-------- Move Servos & collect_data functions --------------
------------------------------------------------------------
*/
void move_servo_to_position(Servo &servo, int position)
{
    servo.write(position);
    delay(servo_1degree_time * object_FOV); // Wait for stabilization
}

Point update_point(Point points[], int pan_servoPosition, int tilt_servoPosition, int distance, int index)
{
    if (distance > Sensor_min_range)
    {
        if (points[index].distance == Limit_distance)
        {
            return sphericalToCartesian(pan_servoPosition, tilt_servoPosition, distance);
        }
        else
        {
            return sphericalToCartesian(pan_servoPosition, tilt_servoPosition, (points[index].distance + distance) / 2.0);
        }
    }
    return sphericalToCartesian(pan_servoPosition, tilt_servoPosition, Limit_distance);
}

int calculate_index(int pan_servoPosition, int tilt_servoPosition)
{
    // Determine the pan and tilt steps based on the current angle and the object FOV
    int panIndex = (abs(pan_servoPosition - pan_start_range) / object_FOV);
    int tiltIndex = (abs(tilt_servoPosition - tilt_start_range) / object_FOV);

    // Calculate the total number of tilt steps to determine how tilt affects the index
    int totalTiltSteps = (abs(tilt_end_range - tilt_start_range) / object_FOV) + 1;

    // The index is then the combination of pan and tilt steps
    return panIndex * totalTiltSteps + tiltIndex;
}

void calculate_tilt_range(int &tiltStart, int &tiltEnd, int &tiltStep, int pan_servoPosition, bool reverse)
{
    bool isEvenPanStep = (pan_servoPosition / object_FOV) % 2 == 0;

    if (reverse && pan_servoPosition == pan_end_range)
    {
        tiltStart = isEvenPanStep ? tilt_end_range : tilt_start_range; // Start from the highest tilt angle
        tiltEnd = isEvenPanStep ? tilt_start_range : tilt_end_range;   // Move to the lowest tilt angle
        isEvenPanStep = false;
    }
    else
    {
        tiltStart = isEvenPanStep ? tilt_start_range : tilt_end_range;
        tiltEnd = isEvenPanStep ? tilt_end_range : tilt_start_range;
    }

    tiltStep = (tiltStart < tiltEnd) ? object_FOV : -object_FOV;
}

void print_point_data(Point points[], int index, int pan_servoPosition, int tilt_servoPosition, int distance)
{
    Serial.print("point index : ");
    Serial.print(index);
    Serial.print(", Pan Angle: ");
    Serial.print(pan_servoPosition);
    Serial.print(", Tilt Angle: ");
    Serial.print(tilt_servoPosition);
    Serial.print(", read Distance: ");
    Serial.print(distance);
    Serial.print(", Point: (");
    Serial.print(points[index].x);
    Serial.print(", ");
    Serial.print(points[index].y);
    Serial.print(", ");
    Serial.print(points[index].z);
    Serial.println(")");
}

void move_collect_data(Point points[], Servo &pan_servo, Servo &tilt_servo, int start_point, int end_point, bool print_data)
{
    int index = 0;
    int step = (start_point < end_point) ? object_FOV : -object_FOV;
    bool reverse = (start_point < end_point) ? false : true; // Determine the direction based on start and end values
    int tiltStart, tiltEnd, tiltStep;

    for (int pan_servoPosition = start_point; (step > 0) ? (pan_servoPosition <= end_point) : (pan_servoPosition >= end_point); pan_servoPosition += step)
    {
        bool isEvenPanStep = (pan_servoPosition / object_FOV) % 2 == 0;
        move_servo_to_position(pan_servo, pan_servoPosition);
        calculate_tilt_range(tiltStart, tiltEnd, tiltStep, pan_servoPosition, reverse);

        for (int tilt_servoPosition = tiltStart; isEvenPanStep ? (tilt_servoPosition <= tiltEnd) : (tilt_servoPosition >= tiltEnd); tilt_servoPosition += tiltStep)
        {
            move_servo_to_position(tilt_servo, tilt_servoPosition);
            index = calculate_index(pan_servoPosition, tilt_servoPosition);
            int distance = get_distance();
            points[index] = update_point(points, pan_servoPosition, tilt_servoPosition, distance, index);
            if (print_data)
            {
                print_point_data(points, index, pan_servoPosition, tilt_servoPosition, distance);
            }

            if (distance > Sensor_min_range && distance <= argent_warning_distance)
            {
                points_in_warning_distance++;
                if (points_in_warning_distance > 2)
                {
                    Serial.println(" EMERGENCY ");
                    points_in_warning_distance = 0;
                }
            }
        }
        if (pan_servoPosition == end_point)
        {
            break;
        }
    }
}