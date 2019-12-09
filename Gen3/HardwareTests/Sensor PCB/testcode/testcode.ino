#include "SensorInterface.h"
#include "TesterInterface.h"

Sensors sensors;
#define compareWithValue 100



//////////////////////////DEFINE THE TESTS////////////////////////
void commonSetup() {
    Serial.begin(9600);
}

void testAnalogSetup() {
    commonSetup();

    pinMode(DAC0, OUTPUT);
    Serial.print("Compare value set to: ");
    Serial.println(compareWithValue);
    analogWriteResolution(10);
    analogWrite(DAC0, compareWithValue);
    Serial.println("Printing sensor analog values:");
}

void testAnalogLoop() {
    sensors.readSensorsAnalog();
    sensors.printAnalogReadings();
}

void testDigitalSetup() {
    commonSetup();

    pinMode(DAC0, OUTPUT);
    Serial.print("Compare value set to: ");
    Serial.println(compareWithValue);
    analogWriteResolution(10);
    analogWrite(DAC0, compareWithValue);
    Serial.println("Printing digital o/p:");
}

void testDigitalLoop() {
    sensors.readSensors();
    // sensors.printAnalogReadings();
    sensors.printDigitalReadings();
}

////////////////////////////////////RUN TEST/////////////////////////////////////
TEST(testDigital)