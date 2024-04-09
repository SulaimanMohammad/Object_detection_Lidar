# Object Detection Using LiDAR and DBSCAN Algorithm

## Overview
This repository contains code for an ESP module that collects data from a Time-of-Flight (ToF) distance sensor mounted on a pan-tilt platform. This setup is designed to detect the presence of objects in a 3D environment. The detection range is determined by the range of servo motors, which can be adjusted based on specific application needs.

## Pan-Tilt Platform Mechanics
The pan-tilt platform operates as follows:

1. **Horizontal Pan Movement:** The platform begins by moving horizontally (panning).
2. **Vertical Tilt Scanning:** At each pan angle, the tilt mechanism scans vertically (e.g., 0-180 degrees).
3. **Sequential Scanning:** After completing a vertical scan, the pan angle changes, and the tilt mechanism scans the next section in the reverse order (180-0 degrees).

With each movement, the distance sensor records a reading. Since the sensor measures the distance of the frontal area, the data (distance, angles) can be represented in a polar coordinate system. Each measurement is then mapped to a Cartesian system, oriented relative to a fixed pan-tilt angle (90-90 degrees), and stored in a matrix of points.

## Data Collection
Upon completion of a full surrounding scan, the points matrix is populated with (x, y, z) coordinates. An object is detected when multiple close distance measurements occur at approximate pan and tilt angles.

## Object Detection with DBSCAN
The DBSCAN (Density-Based Spatial Clustering of Applications with Noise) algorithm is employed for detection. It examines all point coordinates, identifying clusters of closely located points. Each cluster signifies an object, indicating multiple points with similar (x, y, z) coordinates are proximate, implying a shared distance and location relative to the sensor.

### 3D Clustering
DBSCAN in this context is a 3D clustering process, using x, y, z coordinates. This ensures objects are recognized based not only on their polar distance from the sensor but also their spatial location.
Noticing that X axes represent distance from sensor and Y (left-right) ,Z (up-down).

###  K-distance method ( dynamic DBSCAN Epsilon)
DBSCAN requires an Epsilon parameter, crucial in determining whether a point belongs to a specific cluster. Given the usage of x, y, z coordinates, three Epsilons (EpsilonX, EpsilonY, EpsilonZ) are necessary. The Epsilons are dynamically determined using the K-distance method, which identifies a 'Knee' signifying notable changes in neighbors points in the dataset.
The difference here that DBSCAN will have multiple (EpsilonX, EpsilonY, EpsilonZ) instead of one set and classify clusters using one set after another.

## Post-Clustering Analysis
After cluster formation, each cluster's characteristics, such as the number of points, center, and minimum distance point, are calculated. These characteristics help identify the object's position and estimate its size relative to the sensor.

## TOPSIS for merging clusters
### Sweeping mechanism
1. **Sweep1:** The initial horizontal movement from start to end point.
2. **Sweep2:** A verification sweep; for points in Sweep2, the average distance from Sweep1 and Sweep2 is considered.

### TOPSIS
Post Sweep2, another DBSCAN is performed. Due to potential discrepancies from sensor accuracy, it's necessary to merge clusters from both sweeps. This merging uses the TOPSIS (Technique for Order Preference by Similarity to Ideal Solution) method, a multi-criteria decision-making approach. Cluster characteristics serve as criteria for merging, with more weight given to certain aspects like cluster centers. Clusters with proximate centers are likely the same object detected in both sweeps.

## Adaptation and Customization for Specific Applications

This section outlines how to adapt and customize the system for specific application needs. It details the adjustable parameters and provides guidance on how to modify them.
In the `Characterization.h` file, all constants used in the process are defined. Parameters marked with "Modifiable" can be changed as per the application requirements.

### 1. Sensor Section
   - **FOV (Field of View):** Depends on the sensor used and determines the window within which the sensor measures distance.
   - **Object_FOV:** Its the actual movement of servo motors, if the FOV is so small then scanning the full area will take long time, so 3 times of FOV will be enough where the clustring methode will handel connecting the points, but this scale cand be changed.
   - **Min/Max Distance:** Defined by the sensor's capability. The minimum distance should be where readings remain linear.
   - **numSamples:** For some sensors, multiple readings are required. The median of these readings is taken for greater precision.

### 2. Pan-Tilt Range
Defines the start and end angles of the servos, essentially determining the volume of the sphere to be scanned. This section also includes settings for the pin numbers to which the servo motor controls are attached.

### 3. DBSCAN Parameters
   - **minPoints:** The only modifiable parameter here. It represents the minimum number of points required close to each other to form a cluster.

### 4. TOPSIS Parameters
   - Allows setting the weight (importance) of each criterion for merging clusters.

## Compatibility with Different Sensors
There are 2 different implementation used the defulte is [TFMini Plus](https://www.sparkfun.com/products/15179) the second ([VL53L4CX ToF](https://www.adafruit.com/product/5425), which operates via I2C) . To use a different sensor module:

   - Include the library of your new sensor.
   - Change the implementation of `configure_TOF` and `sensor_measurement` functions without altering their signatures.

By following these steps, you can adapt the system to different sensors and customize parameters to fit specific application needs.
