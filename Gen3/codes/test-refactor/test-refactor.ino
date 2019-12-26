#include "src/TesterInterface/TesterInterface.h"
#include "src/SensorInterface/SensorInterface.h"
#include "src/Array/Array.h"

Sensors sensors; 

void setup() {
    Debug::begin();
    Debug::useBluetooth();

    sensors.calibrate();
}

void loop() {
}