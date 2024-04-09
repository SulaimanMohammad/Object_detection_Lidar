#include "IdentifyObjects.h"

/*
----------------------------------------------------------------
---------------------- Info of Objects ------------------------
----------------------------------------------------------------
*/
std::vector<Object> define_objects(std::vector<ClusterInfo> clusters, std::map<int, std::vector<Point>> clusterMap)
{
    std::vector<Object> objects;

    for (const auto &cluster : clusters)
    {
        Object obj(cluster.id, cluster.centroidX, cluster.centroidY, cluster.centroidZ);
        objects.push_back(obj);
    }

    for (const auto &pair : clusterMap)
    {
        int clusterId = pair.first;
        const auto &points = pair.second;

        // Initialize min and max values for x, y, z
        float minX = std::numeric_limits<int>::max();
        float maxX = std::numeric_limits<int>::min();
        float minY = std::numeric_limits<int>::max();
        float maxY = std::numeric_limits<int>::min();
        float minZ = std::numeric_limits<int>::max();
        float maxZ = std::numeric_limits<int>::min();
        for (const auto &point : points)
        {
            // Update min and max for x
            minX = std::min(minX, point.x);
            maxX = std::max(maxX, point.x);

            // Update min and max for y
            minY = std::min(minY, point.y);
            maxY = std::max(maxY, point.y);

            // Update min and max for z
            minZ = std::min(minZ, point.z);
            maxZ = std::max(maxZ, point.z);
        }
        // Calculate distances
        float distanceX = maxX - minX;
        float distanceY = maxY - minY;
        float distanceZ = maxZ - minZ;

        // Find the Object with the corresponding clusterId and update lengths
        for (Object &obj : objects)
        {
            if (obj.id == clusterId)
            {
                obj.setLengths(distanceX, distanceY, distanceZ);
                break; // object is found and updated, exit
            }
        }
    }

    return objects;
}

void printObjects(const std::vector<Object> &objects)
{
    for (const auto &obj : objects)
    {
        Serial.print("Object ID: ");
        Serial.println(obj.id);
        Serial.print("Centroid Coordinates: X = ");
        Serial.print(obj.centerX);
        Serial.print(", Y = ");
        Serial.print(obj.centerY);
        Serial.print(", Z = ");
        Serial.println(obj.centerZ);
        Serial.print(" Width(distance) X = ");
        Serial.print(obj.lengthX);
        Serial.print(",Length Y = ");
        Serial.print(obj.lengthY);
        Serial.print(", Hight Z = ");
        Serial.println(obj.lengthZ);
        Serial.println();
    }
}

void print_position_from_sensor(const std::vector<ClusterInfo> clusters)
{

    for (const auto &cluster : clusters)
    {
        Serial.print("Position: ");
        // Analyze Forward-Backward Position
        if (cluster.centroidX > 0)
        {
            Serial.print("Front");
        }
        else if (cluster.centroidX < 0)
        {
            Serial.println("Back");
        }
        else
        {
            Serial.println("Center");
        }

        Serial.print(" - ");

        // Analyze Left-Right Position
        if (cluster.centroidY > 0)
        {
            Serial.print("Left");
        }
        else if (cluster.centroidY < 0)
        {
            Serial.print("Right ");
        }
        else
        {
            Serial.print("Center");
        }

        Serial.print(" - ");
        // Analyze Up-Down Position
        if (cluster.centroidZ > 0)
        {
            Serial.print("Up");
        }
        else if (cluster.centroidZ < 0)
        {
            Serial.print("Down");
        }
        else
        {
            Serial.print("Center");
        }

        Serial.println();
    }
}
