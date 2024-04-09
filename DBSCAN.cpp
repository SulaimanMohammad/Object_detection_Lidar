#include "DBSCAN.h"
#include "ClusterInfo.h"
#include "Characterization.h"

float epsilonX = 50;
float epsilonY = 50;
float epsilonZ = 50;
/*
-------------------------------------------------------------
--------------------------- DBSCAN --------------------------
-------------------------------------------------------------
*/
std::map<int, std::vector<Point>> detect_objects_clustering(Point points[])
{

    std::vector<std::vector<float>> Epsilons;
    Epsilons = calculateKDistance_set_Epsilon(points, epsilonX, epsilonY, epsilonZ);
    std::map<int, std::vector<Point>> clusterMap;
    int clusterId = 0; // shared between all DBSCAN run so it can be incremented over esplilon sets
    for (size_t i = 0; i < Epsilons.size(); ++i)
    {
        // Accessing the i-th element of each epsilon vector
        epsilonX = Epsilons[i][0];
        epsilonY = Epsilons[i][1];
        epsilonZ = Epsilons[i][2];
        Serial.print(" epsilonX: ");
        Serial.print(epsilonX);
        Serial.print("   epsilonY: ");
        Serial.print(epsilonY);
        Serial.print("  epsilonZ,");
        Serial.println(epsilonZ);

        if (numPoints > 0)
        {
            // The Noise point from one set of Epsilon became a unclassified to be used so they can be clustered by next set
            for (int j = 0; j < numPoints; ++j)
            {
                if (points[j].clusterId == NOISE)
                    points[j].clusterId = UNCLASSIFIED;
            }
            // Apply DBSCAN for clustering only on valid points
            DBSCAN(points, clusterId);
        }
        else
        {
            Serial.println("No valid data points collected.");
        }
    }
        // Populate cluster map with points (it should be done only after all clusters are formed or it will have same repeated cluster after each iteration)
        for (int j = 0; j < numPoints; ++j)
        {
            int clusterId = points[j].clusterId;
            if (clusterId > 0)
            {
                clusterMap[clusterId].push_back(points[j]);
            }
        }
    return clusterMap;
}

int get_number_of_clusters(std::map<int, std::vector<Point>> clusterMap)
{
    return clusterMap.size();
}

bool arePointsNeighbors(const Point &p1, const Point &p2)
{

    return abs(p1.x - p2.x) <= epsilonX && abs(p1.y - p2.y) <= epsilonY && abs(p1.z - p2.z) <= epsilonZ;
}

/*
The reason this function use (points[i].clusterId == NOISE || points[i].clusterId == UNCLASSIFIED )
Suppose first set of epsilon(x,y,z) built a cluster
Then a nother set of esplilon start searching then it will start to dins the core point  see DBSCAN
Now finding core point will use getNeighbors and based on the size the core poitn found if size > minpoint
But the problem if check all points there is possibility to creat a core poitn with another points were already classified in another cluster
and that is not right.
The solution is to search with the new set for the points that are not classefied or consider noise whcih means form a clusters from rest of points,
and notice Noise should still be considered since many cores can be found with one set of espsilon, so some points are noise for one but are included into another
*/

void getNeighbors_multi_espsilon(Point points[], int pointIndex, std::vector<int> &neighbors)
{
    neighbors.clear();
    for (int i = 0; i < numPoints; ++i)
    {
        if (arePointsNeighbors(points[pointIndex], points[i]) && (points[i].clusterId == NOISE || points[i].clusterId == UNCLASSIFIED ) )
        {
            neighbors.push_back(i);
        }
    }
}

void getNeighbors(Point points[], int pointIndex, std::vector<int> &neighbors)
{
    neighbors.clear();
    for (int i = 0; i < numPoints; ++i)
    {
        if (arePointsNeighbors(points[pointIndex], points[i]))
        {
            neighbors.push_back(i);
        }
    }
}

void expandCluster(Point points[], int pointIndex, int clusterId, std::vector<int> &neighbors)
{
    points[pointIndex].clusterId = clusterId;

    for (int i = 0; i < neighbors.size(); ++i)
    {
        int currNeighbor = neighbors[i];
        if (points[currNeighbor].clusterId == UNCLASSIFIED || points[currNeighbor].clusterId == NOISE)
        {
            points[currNeighbor].clusterId = clusterId;

            std::vector<int> newNeighbors;
            getNeighbors_multi_espsilon(points, currNeighbor, newNeighbors);

            for (int newNeighbor : newNeighbors)
            {
                // Check if new neighbor is already in neighbors vector
                if (std::find(neighbors.begin(), neighbors.end(), newNeighbor) == neighbors.end())
                {
                    neighbors.push_back(newNeighbor);
                }
            }
        }
    }
}

void DBSCAN(Point points[], int &clusterId)
{
    for (int i = 0; i < numPoints; ++i)
    {
        if (points[i].distance == 0 || points[i].distance == Limit_distance || points[i].clusterId != UNCLASSIFIED)
        {
            continue; // Skip if the point doesn't meet the criteria or already classified
        }
        // Find core point
        std::vector<int> neighbors;
        getNeighbors_multi_espsilon(points, i, neighbors);

        if (neighbors.size() < minPoints)
        {
            points[i].clusterId = NOISE;
        }
        else
        {

            clusterId++; // Found a new core point, increment clusterId for a new cluster
            points[i].clusterId = clusterId;
            expandCluster(points, i, clusterId, neighbors);
        }
    }
}
