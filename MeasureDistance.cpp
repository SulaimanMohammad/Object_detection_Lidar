#include "MeasureDistance.h"

// Function to calculate the median of an array of integers
int calculateMedianDistance(int samples[])
{
    // Sort the array of distances
    sortArray(samples, numSamples);

    // Compute the median
    if (numSamples % 2 != 0)
    { // Odd number of elements
        return samples[numSamples / 2];
    }
    else
    { // Even number of elementsa
        return (samples[numSamples / 2 - 1] + samples[numSamples / 2]) / 2;
    }
}

// Function to sort an array of integers
void sortArray(int arr[], int numElements)
{
    for (int i = 0; i < numElements - 1; i++)
    {
        for (int j = 0; j < numElements - i - 1; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                // Swap arr[j] and arr[j+1]
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

int get_distance()
{
    int samples[numSamples];
    int validSamples = 0;
    long sumDistance = 0;
    int unvalid_counter = 0;
    for (int i = 0; i < numSamples; i++)
    {
        samples[i] = sensor_measurement();
        if (samples[i] < Sensor_max_range && samples[i] != -1)
        {
            sumDistance += samples[i];
            validSamples++;
        }
        else
        {
            unvalid_counter++;
        }
        delay(5);
    }

    if (unvalid_counter != numSamples)
    {
        int medianDistance = calculateMedianDistance(samples);
        return medianDistance;
    }
    else
    {
        return -1;
    }
}