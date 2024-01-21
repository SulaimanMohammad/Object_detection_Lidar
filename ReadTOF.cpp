#include "ReadTOF.h"

VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, 2);

void configure_TOF()
{
    DEV_I2C.begin();                                 // Initialize I2C bus.
    sensor_vl53l4cx_sat.begin();                     // Configure VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_Off();              // Switch off VL53L4CX satellite component.
    sensor_vl53l4cx_sat.InitSensor(0x12);            // Initialize VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement(); // Start Measurements
}

int sensor_measurement()
{
    VL53L4CX_MultiRangingData_t MultiRangingData;
    VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0, j;
    int status;
    int chosenDistance = INT_MAX;
    int chosenStatus = -1;
    int measurement = 0;
    while (no_of_object_found == 0)
    {
        do
        {
            status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
        } while (!NewDataReady);

        if ((!status) && (NewDataReady != 0))
        {
            status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
            no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
            for (j = 0; j < no_of_object_found; j++)
            {
                int currentDistance = pMultiRangingData->RangeData[j].RangeMilliMeter;
                int currentStatus = pMultiRangingData->RangeData[j].RangeStatus;
                // if ((currentStatus == 0 || currentStatus == 2 || currentStatus == 3 || currentStatus == 4 || currentStatus == 6 || currentStatus == 7) && currentDistance >= 0 && currentDistance < chosenDistance)
                if (currentDistance > 0 && currentDistance < chosenDistance)

                {
                    chosenDistance = currentDistance;
                    chosenStatus = currentStatus;
                }
            }

            if (chosenStatus != -1)
            {

                measurement = chosenDistance;
            }
            else
            {
                measurement = -1;
            }

            if (status == 0)
            {
                status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
            }
        }
    }
    return measurement;
}
