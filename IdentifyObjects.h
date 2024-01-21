#ifndef IDENTIFYOBJECTS_H
#define IDENTIFYOBJECTS_H
#include "ClusterInfo.h"
#include "Coordination.h"

struct Object
{
    int id;
    float centerX, centerY, centerZ;
    float lengthX, lengthY, lengthZ;
    // Constructor to set the id and centroid coordinates
    Object(int objectId, float cx, float cy, float cz)
        : id(objectId), centerX(cx), centerY(cy), centerZ(cz),
          lengthX(0), lengthY(0), lengthZ(0) {}
    // Method to set lengths
    void setLengths(float lx, float ly, float lz)
    {
        lengthX = lx;
        lengthY = ly;
        lengthZ = lz;
    }
};
std::vector<Object> define_objects(std::vector<ClusterInfo> clusters, std::map<int, std::vector<Point>> clusterMap); // Create object and set its info including center and size
void printObjects(const std::vector<Object> &objects);                                                               // print data of each object
void print_position_from_sensor(const std::vector<ClusterInfo> clusters);                                            // print the position of object from the center of pan-tilt

#endif