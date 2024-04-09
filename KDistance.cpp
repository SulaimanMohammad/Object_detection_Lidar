#include "KDistance.h"
/*
---------------------------------------------------------------
---------------------- Find epsilon od DBSCAN -----------------
---------------------------------------------------------------
*/
// Angle-Based Knee Point Detection
int findKneePointIndex(const std::vector<float> &kDistances)
{
    int numkPoints = kDistances.size();
    if (numkPoints < 3)
    {
        return -1; // Not enough points to determine a knee, return invalid index
    }

    // Function to calculate the angle between three points
    auto angle = [](float a, float b, float c) -> float
    {
        float angle = atan2(c - b, 1.0f) - atan2(b - a, 1.0f);
        return fabs(angle);
    };

    // Calculate all angles
    std::vector<float> angles;
    for (int i = 1; i < numkPoints - 1; ++i)
    {
        angles.push_back(angle(kDistances[i - 1], kDistances[i], kDistances[i + 1]));
    }

    // Calculate the average angle change to define a dynamic threshold
    float sumAngleChanges = 0.0;
    for (int i = 1; i < angles.size(); ++i)
    {
        sumAngleChanges += fabs(angles[i] - angles[i - 1]);
    }
    float averageAngleChange = sumAngleChanges / (angles.size() - 1);
    float dynamicThreshold = averageAngleChange * 1;

    // Identify the index of the maximum knee based on dynamic threshold
    float maxAngle = 0.0;
    int kneeIndex = -1;
    for (int i = 0; i < angles.size(); ++i)
    {
        if (i > 0 && fabs(angles[i] - angles[i - 1]) > dynamicThreshold)
        {
            if (angles[i] > maxAngle)
            {
                maxAngle = angles[i];
                kneeIndex = i + 1; // i + 1 because angles are calculated from the second point
            }
        }
    }

    return kneeIndex;
}


/*
---------------------------------------------------------------
------------- K-Distance to define epsilons of DBSCAN ----------
---------------------------------------------------------------
*/
void calculateDimensionKDistance(Point points[], std::vector<float> &kDistances, int dimension)
{
    for (int i = 0; i < numPoints; ++i)
    {
        std::vector<float> distances(numPoints - 1);
        for (int j = 0, idx = 0; j < numPoints; ++j)
        {
            if (i != j)
            {
                float diff = 0;
                switch (dimension)
                {
                case 0:
                    diff = abs(points[i].x - points[j].x);
                    break;
                case 1:
                    diff = abs(points[i].y - points[j].y);
                    break;
                case 2:
                    diff = abs(points[i].z - points[j].z);
                    break;
                }
                distances[idx++] = diff;
            }
        }

        if (distances.size() >= K)
        {
            std::nth_element(distances.begin(), distances.begin() + K - 1, distances.end());
            kDistances[i] = distances[K - 1];
        }
        else
        {
            kDistances[i] = -1;
        }
    }
}

// Function to add to epsilons if index is unique
void addToEpsilonsIfUnique(std::vector<std::vector<float>> &epsilons, const std::vector<float> &kDistancesX, const std::vector<float> &kDistancesY, const std::vector<float> &kDistancesZ, int index, std::set<int> &usedIndices)
{
    if (usedIndices.find(index) == usedIndices.end())
    { // Index not used yet
        if (index == -1)
        {
            epsilons.push_back({10.0, 10.0, 10.0});
        }
        else
        {
            epsilons.push_back({kDistancesX[index], kDistancesY[index], kDistancesZ[index]});
        }

        usedIndices.insert(index); // Mark index as used
    }
}

std::vector<std::vector<float>> calculateKDistance_set_Epsilon(Point points[], float &epsilonX, float &epsilonY, float &epsilonZ)
{
    std::vector<float> kDistancesX(numPoints), kDistancesY(numPoints), kDistancesZ(numPoints);

    calculateDimensionKDistance(points, kDistancesX, 0);
    calculateDimensionKDistance(points, kDistancesY, 1);
    calculateDimensionKDistance(points, kDistancesZ, 2);

    // The most Accurate methode to set values based on Angle-Based Knee Point Detection
    // Find the index of espsilon on  x, y and Z in K-distance
    int index_epsilonX = findKneePointIndex(kDistancesX);
    int index_epsilonY = findKneePointIndex(kDistancesY);
    int index_epsilonZ = findKneePointIndex(kDistancesZ);

    /*
    Generate the combination of Espslion sets as follow
    index of espsilonX then first set of espdilons vector
    1- k-distanceX[index of espsilonX ] , k-distanceY[index of espsilonX ], k-distanceZ[index of espsilonX ]
    index of espsilonY then first set of espdilons vector
    2- k-distanceX[index of espsilonY ] , k-distanceY[index of espsilonY ], k-distanceZ[index of espsilonY ]
    index of espsilonZ then first set of espdilons vector
    3- k-distanceX[index of espsilonZ ] , k-distanceY[index of espsilonZ ], k-distanceZ[index of espsilonZ ]

    And the reason of this compinasion, that finding for example epsilon on X , then the rest should accord with this espdilonX
    For that the same point on distanceY,Z are considered in the vector
    */
    Serial.println(kDistancesZ[index_epsilonZ]);
    std::vector<std::vector<float>> epsilons;
    std::set<int> usedIndices;

    // This function is used to avoid repeated vectors( like in case same indexs on multiple axes like index espdilonX and espsilonY has same index)
    addToEpsilonsIfUnique(epsilons, kDistancesX, kDistancesY, kDistancesZ, index_epsilonX, usedIndices);
    addToEpsilonsIfUnique(epsilons, kDistancesX, kDistancesY, kDistancesZ, index_epsilonY, usedIndices);
    addToEpsilonsIfUnique(epsilons, kDistancesX, kDistancesY, kDistancesZ, index_epsilonZ, usedIndices);

    return epsilons;
}

void print_kDistance(std::vector<float> kDistances)
{
    for (int i = 0; i < numPoints; ++i)
    {
        if (kDistances[i] != -1)
        {
            Serial.print("Point ");
            Serial.print(i);
            Serial.print(" k-distance: ");
            Serial.println(kDistances[i]);
        }
    }
}
