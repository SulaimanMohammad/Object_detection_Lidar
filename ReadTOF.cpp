#include "ReadTOF.h"

// Implementation of TFMini Plus
#include <HardwareSerial.h>
HardwareSerial SerialTFMini(Sensor_serial_port);
void configure_TOF()
{
  SerialTFMini.begin(115200, SERIAL_8N1, RXD2, TXD2); // Initializing serial port;
}

void getTFminiData(int *distance, int *strength)
{
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (SerialTFMini.available())
  {
    rx[i] = SerialTFMini.read();
    if (rx[0] != 0x59)
    {
      i = 0;
    }
    else if (i == 1 && rx[1] != 0x59)
    {
      i = 0;
    }
    else if (i == 8)
    {
      for (j = 0; j < 8; j++)
      {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256))
      {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    }
    else
    {
      i++;
    }
  }
}

int sensor_measurement()
{
  // The sensor works with frame 1000hz, so 1 data per millissecond
  // the time of moving the pan-tilt is ( servo_1degree_time * object_FOV ) millissecond
  // so there is need to read this amount of data to have the most recent value

  for (int i = 0; i < (servo_1degree_time * object_FOV); i++)
  {
    int distance = -1;
    int strength = 0;
    getTFminiData(&distance, &strength);
  }

  int distance = -1;
  int strength = 0;
  int verfiry_counter = 0;
  delay(2);
  getTFminiData(&distance, &strength);
  while (distance == -1)
  {
    getTFminiData(&distance, &strength);
    if (distance != -1 && distance != 0)
    {
      return distance * 10;
    }

    // check that is really zero (undefined reading)
    if (distance == 0 && verfiry_counter <= 10)
    {
      delay(2);
      distance = -1; // to read back and check that reading is really 0
      verfiry_counter++;
    }
    else if (distance == 0 && verfiry_counter > 10)
      return -1;
  }
}

// Implementation of VL53L4CX
/*
#include <Wire.h>
#include <assert.h>
#include <vl53l4cx_class.h>
#define DEV_I2C Wire
extern VL53L4CX sensor_vl53l4cx_sat; // Instance of the sensor

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
*/
