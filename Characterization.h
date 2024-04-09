#ifndef CHARACTERIZATION_H
#define CHARACTERIZATION_H

#define SerialPort Serial
#define argent_warning_distance 400 // Modifiable
/* ---------- Lidar variables ---------- */
#define FOV 3                 // Field of View of the sensor, Modifiable
#define Sensor_max_range 12000 // Maximum reading the sensor give, here 6m in mm, Modifiable
#define Sensor_min_range 20    // Modifiable
#define numSamples 1          // Number of samples for one reading, Modifiable
#define Sensor_serial_port 2   // Serial port for the sensor, Modifiable
#define RXD2 16
#define TXD2 17
/* ----------  Pan-tilt range ---------- */
#define pan_start_range 0 // Modifiable
#define pan_end_range 120  // Modifiable
#define tilt_start_range 0 // Modifiable
#define tilt_end_range 120 // Modifiable

#define object_FOV (FOV * 2) // Field of object detection (enough angles to detect object)
#define pan_active_range (pan_end_range - pan_start_range)
#define tilt_active_range (tilt_end_range - tilt_start_range)
#define MAX_POINTS (((pan_active_range / object_FOV) + 1) * ((tilt_active_range / object_FOV) + 1))
#define Limit_distance Sensor_max_range - (Sensor_max_range * 0.20)
#define numPoints MAX_POINTS

/* ---------- Servo variables ---------- */
#define pan_controle 2       // Modifiable
#define tilt_controle 4      // Modifiable
#define servo_1degree_time 3 // Modifiable, milliseconds

/* ---------- DBSCAN parameters ---------- */
#define minPoints object_FOV // Number of close point to form cluster, Modifiable
#define NOISE -2
#define UNCLASSIFIED -1
#define K minPoints // K-Distance parameter= minPoints of DBSCAN

/* ---------- TOPSIS parameters ---------- */
#define weightCorePointDistance 0.31 // Modifiable
#define weightNumberOfPoints 0.09    // Modifiable
#define weightMinDistance 0.1        // Modifiable
#define weightCenterDistance 0.5     // Modifiable
#define shift_margin_to_merge 300    // Max diffeence in distance between two clusters core to merge them, Modifiable

// First merge between 2 sweeps
#define scoreMergeThreshold 0.05   // Define a threshold for merging clusters based on TOPSIS score similarity,  // Modifiable
#define distanceMergeThreshold 200 // Modifiable
// Second merge of the result of first one for general view of objects
#define PostMergeTOPSIS_scoreThreshold 0.02   // Modifiable
#define PostMergeTOPSIS_distanceThreshold 100 // Modifiable

#endif
