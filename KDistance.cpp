#include "KDistance.h"
/*
---------------------------------------------------------------
---------------------- Find epsilon od DBSCAN -----------------
---------------------------------------------------------------
*/
// Elbow Method
float findElbowPoint(const std::vector<float> &kDistances)
{
    int numkPoints = kDistances.size();
    std::vector<int> elbowIndices; // Store indices of all elbow points

    for (int i = 1; i < numkPoints - 1; ++i)
    {
        // Finding local maxima in the k-distance graph
        if (kDistances[i] > kDistances[i - 1] && kDistances[i] > kDistances[i + 1])
        {
            elbowIndices.push_back(i);
        }
    }

    // Calculate the average of all elbow points
    float sumElbowDistances = 0;
    for (int index : elbowIndices)
    {
        sumElbowDistances += kDistances[index];
    }

    float averageElbowDistance = 0;
    if (!elbowIndices.empty())
    {
        averageElbowDistance = sumElbowDistances / elbowIndices.size();
    }

    return averageElbowDistance;
}

float findKneePoint_simple(const std::vector<float> &kDistances)
{
    int numkPoints = kDistances.size();
    std::vector<int> kneeIndices; // Store indices of all knee points

    for (int i = 1; i < numkPoints - 1; ++i)
    {
        // Finding points with significant change in slope
        float diff1 = kDistances[i] - kDistances[i - 1];
        float diff2 = kDistances[i + 1] - kDistances[i];
        if (diff1 * diff2 < 0)
        { // Change in the sign of the slope
            kneeIndices.push_back(i);
        }
    }

    // Calculate the average of all knee points
    float sumKneeDistances = 0;
    for (int index : kneeIndices)
    {
        sumKneeDistances += kDistances[index];
    }

    float averageKneeDistance = 0;
    if (!kneeIndices.empty())
    {
        averageKneeDistance = sumKneeDistances / kneeIndices.size();
    }

    return averageKneeDistance;
}

// Angle-Based Knee Point Detection
float findKneePoint(const std::vector<float> &kDistances)
{
    int numkPoints = kDistances.size();
    if (numkPoints < 3)
    {
        return -1; // Not enough points to determine a knee
    }

    // Function to calculate the angle between three points
    auto angle = [](float a, float b, float c) -> float
    {
        float angle = atan2(c - b, 1.0f) - atan2(b - a, 1.0f);
        return fabs(angle);
    };

    float maxAngle = 0.0;
    int kneeIndex = -1;

    // Iterate over the points to find the maximum angle
    for (int i = 1; i < numkPoints - 1; ++i)
    {
        float currentAngle = angle(kDistances[i - 1], kDistances[i], kDistances[i + 1]);
        if (currentAngle > maxAngle)
        {
            maxAngle = currentAngle;
            kneeIndex = i;
        }
    }

    return (kneeIndex != -1) ? kDistances[kneeIndex] : -1;
}

int estimateNoisePoints(Point points[], float thresholdX, float thresholdY, float thresholdZ)
{
    int noisePointCount = 0;
    for (int i = 0; i < numPoints; ++i)
    {
        int neighborCount = 0;
        for (int j = 0; j < numPoints; ++j)
        {
            if (i != j &&
                abs(points[i].x - points[j].x) <= thresholdX &&
                abs(points[i].y - points[j].y) <= thresholdY &&
                abs(points[i].z - points[j].z) <= thresholdZ)
            {
                neighborCount++;
            }
        }
        if (neighborCount < minPoints)
        {
            noisePointCount++;
        }
    }
    return noisePointCount;
}

void adjustThresholdsBasedOnFeedback(Point points[], std::vector<float> kDistancesX, std::vector<float> kDistancesY, std::vector<float> kDistancesZ, float &epsilonX, float &epsilonY, float &epsilonZ)
{

    // Calculate elbow and knee points for each dimension
    float elbowPointX = findElbowPoint(kDistancesX);
    float elbowPointY = findElbowPoint(kDistancesY);
    float elbowPointZ = findElbowPoint(kDistancesZ);

    float kneePointX = findKneePoint(kDistancesX);
    float kneePointY = findKneePoint(kDistancesY);
    float kneePointZ = findKneePoint(kDistancesZ);

    Serial.print("Elbow Point X: ");
    Serial.print(elbowPointX);
    Serial.print(" ,Elbow Point Y: ");
    Serial.print(elbowPointY);
    Serial.print(" ,Elbow Point Z: ");
    Serial.println(elbowPointZ);

    Serial.print("Knee Point X: ");
    Serial.print(kneePointX);
    Serial.print(" ,Knee Point Y: ");
    Serial.print(kneePointY);
    Serial.print(" ,Knee Point Z: ");
    Serial.println(kneePointZ);

    // Find the combination that produces the minimum noise for each dimension
    std::vector<float> epsilonCandidatesX = {elbowPointX, kneePointX};
    std::vector<float> epsilonCandidatesY = {elbowPointY, kneePointY};
    std::vector<float> epsilonCandidatesZ = {elbowPointZ, kneePointZ};

    float minNoiseX = std::numeric_limits<float>::max();
    float minNoiseY = std::numeric_limits<float>::max();
    float minNoiseZ = std::numeric_limits<float>::max();

    // Optimal thresholds
    float optimalEpsilonX = epsilonCandidatesX[0];
    float optimalEpsilonY = epsilonCandidatesY[0];
    float optimalEpsilonZ = epsilonCandidatesZ[0];

    for (float candidateX : epsilonCandidatesX)
    {
        for (float candidateY : epsilonCandidatesY)
        {
            for (float candidateZ : epsilonCandidatesZ)
            {
                int noisePoints = estimateNoisePoints(points, candidateX, candidateY, candidateZ);

                // Check for X
                if (candidateX == candidateY && candidateX == candidateZ && noisePoints < minNoiseX)
                {
                    minNoiseX = noisePoints;
                    optimalEpsilonX = candidateX;
                }

                // Check for Y
                if (candidateY == candidateX && candidateY == candidateZ && noisePoints < minNoiseY)
                {
                    minNoiseY = noisePoints;
                    optimalEpsilonY = candidateY;
                }

                // Check for Z
                if (candidateZ == candidateX && candidateZ == candidateY && noisePoints < minNoiseZ)
                {
                    minNoiseZ = noisePoints;
                    optimalEpsilonZ = candidateZ;
                }
            }
        }
    }

    epsilonX = optimalEpsilonX;
    epsilonY = optimalEpsilonY;
    epsilonZ = optimalEpsilonZ;

    Serial.print("Optimal Epsilon X: ");
    Serial.println(epsilonX);
    Serial.print("Optimal Epsilon Y: ");
    Serial.println(epsilonY);
    Serial.print("Optimal Epsilon Z: ");
    Serial.println(epsilonZ);
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

void calculateKDistance_set_Epsilon(Point points[], float &epsilonX, float &epsilonY, float &epsilonZ)
{
    std::vector<float> kDistancesX(numPoints), kDistancesY(numPoints), kDistancesZ(numPoints);

    calculateDimensionKDistance(points, kDistancesX, 0);
    calculateDimensionKDistance(points, kDistancesY, 1);
    calculateDimensionKDistance(points, kDistancesZ, 2);

    // Adjust Thresholds Based On Feedback to choose the value of threshould ( depends on application)
    // adjustThresholdsBasedOnFeedback(kDistancesX, kDistancesY, kDistancesZ, epsilonX, epsilonY,epsilonZ);

    // The most Accurate methode to set values based on Angle-Based Knee Point Detection
    epsilonX = findKneePoint(kDistancesX);
    epsilonY = findKneePoint(kDistancesY);
    epsilonZ = findKneePoint(kDistancesZ);
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
