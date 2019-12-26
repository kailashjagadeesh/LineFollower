#include "SensorInterface.h"

#include <Arduino.h>

#include "../LEDInterface/LEDInterface.h"
#include "../PushButtonInterface/PushButtonInterface.h"
#include "../TesterInterface/TesterInterface.h"

#ifndef NUM_SENSORS
#define NUM_SENSORS 9
#endif

#ifndef NUM_PIDSENSORS
#define NUM_PIDSENSORS 5
#endif

#ifndef SENSOR_CENTER_PINS
#define SENSOR_CENTER_PINS {47, 49}
#define NUM_SENSOR_CENTER_PINS 2
#endif

///overshoot handling
bool Sensors::overshootCompleted() {
    return (overshootData.index == -1);
}

void Sensors::addLeftOvershoot() {
    overshootData.push(overshootData.left);
}

void Sensors::addRightOvershoot() {
    overshootData.push(overshootData.right);
}

void Sensors::waitForOvershoot() {
    while (!overshootCompleted()) {
        overshootControl();
    }
}

void Sensors::overshootControl() {
    uint8_t currentState = readRearSensors();
    uint8_t currentLeft = currentState & 0b10;
    uint8_t currentRight = currentState & 0b01;

    uint8_t prevLeft = overshootData.prevState & 0b10;
    uint8_t prevRight = overshootData.prevState & 0b01;

    if (((overshootData.nextOvershoot() == overshootData.left) && (prevLeft - currentLeft < 0))
        || ((overshootData.nextOvershoot() == overshootData.right) && (prevRight - currentRight < 0))) {
        overshootData.pop();
    } 

    overshootData.prevState = currentState;
}

//pin definitions
const uint8_t Sensors::analogPins[12] = {A8, A7, A6, A5, A4, A3, A2, A1, A0, A9, A10, A11};
const uint8_t Sensors::digitalPins[12] = {6, 5, 4, 3, 36, 28, 38, 26, 32, 34, 30, 40};
const uint8_t Sensors::sensorCenterPins[12] = SENSOR_CENTER_PINS;

uint8_t Sensors::readRearSensors() {
    rearSensorStatus = 0;
#ifndef WHITELINE_LOGIC
    if (analogRead(analogPins[9]) > calibratedValues.thresholdValues[9])
        rearSensorStatus |= 0b10;
    if (analogRead(analogPins[10]) > calibratedValues.thresholdValues[10]) 
        rearSensorStatus |= 0b01;
#else
    if (analogRead(analogPins[9]) <= calibratedValues.thresholdValues[9])
        rearSensorStatus |= 0b10;
    if (analogRead(analogPins[10] <= calibratedValues.thresholdValues[10])) 
        rearSensorStatus |= 0b01;
#endif

return rearSensorStatus;
}

void Sensors::readCenterSensors() {
    for (int i = 0; i < NUM_SENSOR_CENTER_PINS; ++i)
    #ifdef WHITELINE_LOGIC
        digitalValues |= (!digitalRead(sensorCenterPins[i]) << (NUM_SENSORS/2));
    #else 
        digitalValues |= (digitalRead(sensorCenterPins[i]) << (NUM_SENSORS/2));
    #endif
}

void Sensors::printDigitalValues()
{
    Debug::print("Converted values: ");
    Debug::println(digitalValues, BIN);
}

void Sensors::printCalibratedInfo()
{
    Debug::println("Calibrated values: \n");
    Debug::println("WHITE:");
    Debug::print("MAX VALUES:\n\t");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        Debug::print(whiteValues.highValues[i]);
        Debug::print("\t");
    }
    Debug::print("\nMIN VALUES:\n\t");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        Debug::print(whiteValues.lowValues[i]);
        Debug::print("\t");
    }

    Debug::println("\n\n\nBLACK:");
    Debug::print("MAX VALUES:\n\t");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        Debug::print(blackValues.highValues[i]);
        Debug::print("\t");
    }
    Debug::print("\nMIN VALUES:\n\t");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        Debug::print(blackValues.lowValues[i]);
        Debug::print("\t");
    }

    Debug::println("\n\nTHRESHOLD VALUES:\n\t");
    Debug::print("THRESHOLD VALUES:\n");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        Debug::print(calibratedValues.thresholdValues[i]);
        Debug::print("\t");
    }
}

void Sensors::printAnalogReadings()
{
    Debug::print("\n\nAnalog: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Debug::print(analogReadings[i]);
        Debug::print("\t");
    }
}

void Sensors::printDigitalReadings()
{

    Debug::print("\n\nDigital: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Debug::print(digitalReadings[i]);
        Debug::print("\t");
    }
}

void Sensors::calibrate()
{
    //clear the readings
    for (int i = 0; i < NUM_SENSORS +2; i++)
    {
        whiteValues.highValues[i] = whiteValues.lowValues[i] = 0;
        blackValues.highValues[i] = blackValues.lowValues[i] = 0;
        calibratedValues.thresholdValues[i] = 0;
    }
    LED::write(0, HIGH);
    LED::write(1, HIGH);

    Debug::println("Keep on white surface");
    PushButtonInterface::waitForButton(0);

    for (int i = 0; i < 100; i++)
    {
        readSensorsAnalog();
        for (int j = 0; j < NUM_SENSORS+2; ++j)
        {
            if (analogReadings[j] > whiteValues.highValues[j])
                whiteValues.highValues[j] = analogReadings[j];

            if (i == 0 || analogReadings[j] < whiteValues.lowValues[j])
                whiteValues.lowValues[j] = analogReadings[j];
        }
    }

    LED::write(0, LOW);

    Debug::println("Keep on black surface");
    PushButtonInterface::waitForButton(0);
    
    for (int i = 0; i < 100; i++)
    {
        readSensorsAnalog();
        for (int j = 0; j < NUM_SENSORS+2; ++j)
        {
            if (analogReadings[j] > blackValues.highValues[j])
                blackValues.highValues[j] = analogReadings[j];

            if (i == 0 || analogReadings[j] < blackValues.lowValues[j])
                blackValues.lowValues[j] = analogReadings[j];
        }
    }

    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        whiteValues.averageValues[i] = (whiteValues.highValues[i] + whiteValues.lowValues[i]) / 2;
        blackValues.averageValues[i] = (blackValues.highValues[i] + blackValues.lowValues[i]) / 2;
        calibratedValues.thresholdValues[i] = (whiteValues.lowValues[i] + blackValues.highValues[i]) / 2;
    }

    calibratedValues.thresholdValues[9] = (whiteValues.highValues[9] + blackValues.lowValues[9])/2;
    calibratedValues.thresholdValues[10] = (whiteValues.highValues[10] + blackValues.lowValues[10])/2;

    LED::write(1, 0);
}

void Sensors::convertAnalogToDigital()
{
    digitalValues = 0;
    nSensorsOnLine = 0;
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        if (analogReadings[i] > calibratedValues.thresholdValues[i])
        {
            digitalValues |= (1 << (NUM_SENSORS - i - 1));
            nSensorsOnLine ++;
        }
    }

    readCenterSensors();
}

Sensors::Sensors()
{
    //initialize pins
    for (int i = 0; i < NUM_SENSORS+2; i++)
    {
        pinMode(digitalPins[i], INPUT);
        pinMode(analogPins[i], INPUT);
    }

    for (int i = 0; i < NUM_SENSOR_CENTER_PINS; i++) {
        pinMode(sensorCenterPins[i], INPUT);
    }
}

void Sensors::readSensorsAnalog()
{
    for (int i = 0; i < NUM_SENSORS+2; i++)
    {
#ifdef WHITELINE_LOGIC
        analogReadings[i] = analogRead(analogPins[i]);
#else
        analogReadings[i] = 1023 - analogRead(analogPins[i]);
#endif
    }
}
// CF pin must be indexed 8th(count from 0)
void Sensors::readSensorsDigital()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
#ifdef WHITELINE_LOGIC
        digitalReadings[i] = digitalRead(digitalPins[i]);
#else
        digitalReadings[i] = !digitalRead(digitalPins[i]);
#endif
    
    }
}

void Sensors::readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
#ifdef WHITELINE_LOGIC
        digitalReadings[i] = digitalRead(digitalPins[i]);
        analogReadings[i] = analogRead(analogPins[i]);
#else
        digitalReadings[i] = !digitalRead(digitalPins[i]);
        analogReadings[i] = 1023 - analogRead(analogPins[i]);
#endif
    }
} 

//Reads the sensor analog values and returns the position of the line
uint16_t Sensors::readLine()
{
    uint16_t sum[2] = {0, 0};
    readSensorsAnalog();
    convertAnalogToDigital();
    for (int i = (NUM_SENSORS - NUM_PIDSENSORS)/2, j = 0; j < NUM_PIDSENSORS; ++i, ++j)
    {
        if (digitalValues & (1 << (NUM_SENSORS - i - 1)))
        {
            sum[0] += (j) * 1000;
            sum[1] += 1;
        }
    }
    return (sum[1] > 0) ? (sum[0] / sum[1]) : ((NUM_PIDSENSORS - 1) * 1000 / 2);
}