#include "src/TesterInterface/TesterInterface.h"
#include "src/SensorInterface/SensorInterface.h"
#include "src/PushButtonInterface/PushButtonInterface.h"

Sensors sensors; 

void setup() {
    Debug::begin();
    Debug::useBluetooth();

    sensors.calibrate();
    sensors.printCalibratedInfo();
    PushButtonInterface::waitForButton(0);
}

void loop() { 
    Debug::println(sensors.readRearSensors(), 2);
}