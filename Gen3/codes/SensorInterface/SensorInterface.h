#include "ISRs.h"
#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#define NUM_SENSORS 9
#define CLASSIFICATION_THRESHOLD 511
#define CFPin 6

#define LFPin 5
#define LMPin 4
#define LB1Pin 3
#define LB2Pin 40

#define RFPin 28
#define RMPin 30
#define RB1Pin 26
#define RB2Pin 32
class Sensors
{
    //ds to strore the readings
    uint16_t analogReadings[12];
    uint16_t calibratedHighValues[12];
    uint16_t calibratedLowValues[12];

    //pin definitions
    static const uint8_t analogPins[12];
    static const uint8_t digitalPins[12];

public:
    uint16_t digitalReadings[12];
    void attachAllInterrupts();
    //read values
    void readSensorsAnalog();
    void readSensorsDigital(uint8_t*, bool*);
    void readSensors();

    //callibration (set DAC Values)
    void calibrate();

    //get line for PID
    uint16_t readLine();

    Sensors();

    //debug info
    void printDebugInfo();
    void printAnalogReadings();
    void printDigitalReadings();
};

//pin definitions
const uint8_t Sensors::analogPins[12] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
const uint8_t Sensors::digitalPins[12] = {6, 5, 4, 3, 40, 28, 30, 26, 32, 38, 34, 36};
void Sensors::attachAllInterrupts()
{
    attachInterrupt(digitalPinToInterrupt(CFPin), CFISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(LFPin), LFISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LMPin), LMISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LB1Pin), LB1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LB2Pin), LB2ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(RFPin), RFISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RMPin), RMISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RB1Pin), RB1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RB2Pin), RB2ISR, CHANGE);
}
//debug
void Sensors::printDebugInfo()
{
    Serial.println("SENSOR CALIBRATED VALUES [HIGH]:");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(calibratedHighValues[i]);
        Serial.print("\t");
    }

    Serial.println("\n\nSENSOR CALIBRATED VALUES [LOW]:");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(calibratedLowValues[i]);
        Serial.print("\t");
    }
}

void Sensors::printAnalogReadings()
{
    Serial.print("\n\nAnalog: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(analogReadings[i]);
        Serial.print("\t");
    }
}

void Sensors::printDigitalReadings()
{

    Serial.print("\n\nDigital: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(digitalReadings[i]);
        Serial.print("\t");
    }
}

//calibration of the DAC value
void Sensors::calibrate()
{
    //keeping track of number of values
    uint8_t tempH[12], tempL[12];
    for (int i = 0; i < 12; i++)
    {
        tempH[i] = 0;
        tempL[i] = 0;
    }

    //read analog values over 100 times
    for (int i = 0; i < 100; i++)
    {
        readAnalog();

        for (int j = 0; i < NUM_SENSORS; j++)
        {
            if (analogReadings[j] >= CLASSIFICATION_THRESHOLD)
            {
                calibratedHighValues[j] += analogReadings[j];
                tempH[j]++;
            }
            else
            {
                calibratedLowValues[j] += analogReadings[j];
                tempL[j]++;
            }
        }

        //delay between readings
        delay(10);
    }

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        calibratedHighValues[i] /= tempH[i];
        calibratedLowValues[i] /= tempL[i];
    }

    //setting the DAC values of the average of the average of high and low means
    uint16_t temp = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        temp += calibratedHighValues[i] + calibratedLowValues[i];
    }

    //analogWrite(DAC0, temp / NUM_SENSORS);
}

Sensors::Sensors()
{
    //initialize pins
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pinMode(digitalPins[i], INPUT);
        pinMode(analogPins[i], INPUT);

        calibratedHighValues[i] = 0;
        calibratedLowValues[i] = 0;
    }
}

void Sensors::readAnalog()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        analogReadings[i] = analogRead(analogPins[i]);
    }
}
// CF pin must be indexed 8th(count from 0)
void Sensors::readSensorsDigital(uint8_t *backSensorState, bool *CFState)
{
    *backSensorState = 0;
    *CFState = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        digitalReadings[i] = digitalRead(digitalPins[i]);
        if (i < 8)
        {
            (*backSensorState) |= digitalReadings[i] << (7 - i);
        }
        else if (i == 8)
        {
            *CFState = digitalReadings[i];
        }
    }
}

void Sensors::readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        digitalReadings[i] = digitalRead(digitalPins[i]);
        analogReadings[i] = analogRead(analogPins[i]);
    }
}

uint16_t Sensors::readLine()
{
    uint16_t sum[2] = {0, 0};
    readSensorsAnalog();
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        sum[0] += analogReadings[i] * i * 1000;
        sum[1] += analogReadings[i];
    }

    return sum[0] / sum[1];
}

#endif