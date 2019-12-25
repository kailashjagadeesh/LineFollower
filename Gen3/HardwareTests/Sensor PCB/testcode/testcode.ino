//CONFIGURATION
#define BLACKLINE_LOGIC
#define SERIALD bluetooth

#include "softwareSerial.h"
Bluetooth bluetooth;

#include "SensorInterface.h"
#include "LEDInterface.h"
#include "TesterInterface.h"
#include "MotorDriverInterface.h"

Sensors sensors;
Motor motors;
#define CALIB_VALUE 200
#define RESOLUTION 8
#define mapAnalogToDac(x) (x) * 255 / 1023

//////////////////////////DEFINE THE TESTS////////////////////////
void commonSetup() {
    bluetooth.begin();
    LED::init();
}

void setupDAC() {
    commonSetup();
    pinMode(DAC0, OUTPUT);
    SERIALD.print("Compare value set to: ");
    SERIALD.println(mapAnalogToDac(CALIB_VALUE));
    analogWriteResolution(RESOLUTION);
    analogWrite(DAC0, mapAnalogToDac(CALIB_VALUE));
}

void testBothSetup() {
    setupDAC();
}

void testBothLoop() {
    sensors.readSensors();
    sensors.printAnalogReadings();
    sensors.printDigitalReadings();
}

void testAnalogSetup() {
    commonSetup();

    pinMode(DAC0, OUTPUT);
    SERIALD.print("Compare value set to: ");
    SERIALD.println(mapAnalogToDac(CALIB_VALUE));
    analogWriteResolution(RESOLUTION);
    analogWrite(DAC0, mapAnalogToDac(CALIB_VALUE));
    SERIALD.println("Printing sensor analog values:");
}

void testAnalogLoop() {
    sensors.readSensorsAnalog();
    sensors.printAnalogReadings();
}

void testDigitalSetup() {
    commonSetup();
    motors.stopMotors();

    pinMode(DAC0, OUTPUT);
    SERIALD.print("Compare value set to: ");
    SERIALD.println(mapAnalogToDac(CALIB_VALUE));
    analogWriteResolution(RESOLUTION);
    analogWrite(DAC0, mapAnalogToDac(CALIB_VALUE));
    SERIALD.println("Printing digital o/p:");
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

    // SERIALD.println("Will print analog values in 10...");
    // delay(10000);
}

void testCalibLoop() {
    // sensors.readSensorsAnalog();
    // sensors.printAnalogReadings();
}

void testConversionSetup() {
    commonSetup();
    sensors.calibrate();
    sensors.printCalibratedInfo();
    
    SERIALD.println("\nBeginning sensors readings in 3 seconds...");
    delay(3000);
}

void testConversionLoop() {
    sensors.readSensors();
    sensors.convertAnalogToDigital();
    sensors.printAnalogReadings();
    sensors.printDigitalValues();
}

void testLineDetectionSetup() {
    commonSetup();
    sensors.calibrate();
}

void testLineDetectionLoop() {
    SERIALD.print("Line position: ");
    sensors.readSensors();
    sensors.convertAnalogToDigital();
    sensors.printDigitalValues();
    SERIALD.println(sensors.readLine());
}

////////////////////////////////////RUN TEST/////////////////////////////////////
TEST(testConversion)