#include "src/LEDInterface/LEDInterface.h"
#include "src/PushButtonInterface/PushButtonInterface.h"
#include "src/SensorInterface/SensorInterface.h"
#include "src/MotorDriverInterface/MotorDriverInterface.h"
#include "src/PIDControl/PIDControl.h"
#include "src/BluetoothInterface/BluetoothInterface.h"
#include "src/UltrasonicInterface/UltrasonicInterface.h"
#include "src/JunctionControl/JunctionControl.h"

Motor motors;
Sensors sensors;
Bluetooth bluetooth;

void setup() {
    LED::init();
    PushButtonInterface::init();
    sensors.calibrate();
    sensors.printCalibratedInfo();
    bluetooth.begin();
}

void loop() {
    sensors.readSensors();
    sensors.convertAnalogToDigital();
    bluetooth.println(sensors.digitalValues);
}