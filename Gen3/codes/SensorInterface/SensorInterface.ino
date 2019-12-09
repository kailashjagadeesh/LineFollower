////////////////////TESTING CODE/////////////////////////
#include "SensorInterface.h"

Sensors sensors;

void setup()
{
    Serial.begin(9600);
    Serial.println("Begin calibration...");
    sensors.calibrate();
    sensors.printCalibratedInfo();
}

void loop()
{
    sensors.readSensors();
    sensors.printSensorReadings();
}