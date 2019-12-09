#include "SensorInterface.h"
#include "TesterInterface.h"
#include "MotorDriverInterface.h"

Sensors sensors;
Motor motors;
#define CALIB_VALUE 0
#define RESOLUTION 8
#define mapAnalogToDac(x) (x) * 255 / 1023

//////////////////////////DEFINE THE TESTS////////////////////////
void commonSetup() {
    Serial.begin(9600);
}

void testAnalogSetup() {
    commonSetup();

    pinMode(DAC0, OUTPUT);
    Serial.print("Compare value set to: ");
    Serial.println(mapAnalogToDac(CALIB_VALUE));
    analogWriteResolution(RESOLUTION);
    analogWrite(DAC0, mapAnalogToDac(CALIB_VALUE));
    Serial.println("Printing sensor analog values:");
}

void testAnalogLoop() {
    sensors.readSensorsAnalog();
    sensors.printAnalogReadings();
}

void testDigitalSetup() {
    commonSetup();
    motors.stopMotors();

    pinMode(DAC0, OUTPUT);
    Serial.print("Compare value set to: ");
    Serial.println(mapAnalogToDac(CALIB_VALUE));
    analogWriteResolution(RESOLUTION);
    analogWrite(DAC0, mapAnalogToDac(CALIB_VALUE));
    Serial.println("Printing digital o/p:");
}

void testDigitalLoop() {
    sensors.readSensors();
    // sensors.printAnalogReadings();
    sensors.printDigitalReadings();
}

void testCalibSetup() {
    commonSetup();
    sensors.calibrate();
    sensors.printCalibratedInfo();

    // Serial.println("Will print analog values in 10...");
    // delay(10000);
}

void testCalibLoop() {
    // sensors.readSensorsAnalog();
    // sensors.printAnalogReadings();
}

////////////////////////////////////RUN TEST/////////////////////////////////////
TEST(testDigital)