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

    //pin definitions
    static const uint8_t analogPins[12];
    static const uint8_t digitalPins[12];

public:
    struct calibratedValues{
        uint16_t highValues[NUM_SENSORS];
        uint16_t lowValues[NUM_SENSORS];
        uint16_t averageValues[NUM_SENSORS];
    } whiteValues, blackValues;

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
    void printCalibratedInfo();
    void printAnalogReadings();
    void printDigitalReadings();
};

//pin definitions
const uint8_t Sensors::analogPins[12]  = {A0, A1, A2, A3, A11, A5, A6, A7, A8, A10,  A9, A4};
const uint8_t Sensors::digitalPins[12] = { 6,  5,  4,  3,  36, 28, 30, 26, 32,  34,  36, 40};
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
void Sensors::printCalibratedInfo()
{
    Serial.println("Calibrated values: \n");
    Serial.println("WHITE:");
    Serial.print("MAX VALUES:\t");
    for (int i = 0 ; i < NUM_SENSORS; ++i)
    {
        Serial.print(whiteValues.highValues[i]);
        Serial.print("\t");
    }
    Serial.print("\nMIN VALUES:\t");
    for (int i = 0 ; i < NUM_SENSORS; ++i)
    {
        Serial.print(whiteValues.lowValues[i]);
        Serial.print("\t");
    }
    Serial.print("\nAVG VALUES:\t");
    for (int i = 0 ; i < NUM_SENSORS; ++i)
    {
        Serial.print(whiteValues.averageValues[i]);
        Serial.print("\t");
    }

    Serial.println("\n\n\nBLACK:");
    Serial.print("MAX VALUES:\t");
    for (int i = 0 ; i < NUM_SENSORS; ++i)
    {
        Serial.print(blackValues.highValues[i]);
        Serial.print("\t");
    }
    Serial.print("\nMIN VALUES:\t");
    for (int i = 0 ; i < NUM_SENSORS; ++i)
    {
        Serial.print(blackValues.lowValues[i]);
        Serial.print("\t");
    }
    Serial.print("\nAVG VALUES:\t");
    for (int i = 0 ; i < NUM_SENSORS; ++i)
    {
        Serial.print(blackValues.averageValues[i]);
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

void Sensors::calibrate() {
    //clear the readings
    for (int i = 0; i < NUM_SENSORS; i++) {
        whiteValues.highValues[i] = whiteValues.lowValues[i] = 0;
        blackValues.highValues[i] = blackValues.lowValues[i] = 0;
    }

    Serial.println("Keep on white surface");
    delay(5000);
    for (int i = 0; i < 100; i++) {
        readSensorsAnalog();
        for (int j = 0; j < NUM_SENSORS; ++j) {
            if (analogReadings[j] > whiteValues.highValues[j])
                whiteValues.highValues[j] = analogReadings[j];
            
            if (i == 0 || analogReadings[j] < whiteValues.lowValues[j])
                whiteValues.lowValues[j] = analogReadings[j];
        }
    }

    Serial.println("Keep on black surface");
    delay(5000);
    for (int i = 0; i < 100; i++) {
        readSensorsAnalog();
        for (int j = 0; j < NUM_SENSORS; ++j) {
            if (analogReadings[j] > blackValues.highValues[j])
                blackValues.highValues[j] = analogReadings[j];
            
            if (i == 0 || analogReadings[j] < blackValues.lowValues[j])
                blackValues.lowValues[j] = analogReadings[j];
        }
    }

    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        whiteValues.averageValues[i] = (whiteValues.highValues[i] + whiteValues.lowValues[i])/2;
        blackValues.averageValues[i] = (blackValues.highValues[i] + blackValues.lowValues[i])/2;
    }
}

/*
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
        readSensorsAnalog();

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
/**/
Sensors::Sensors()
{
    //initialize pins
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pinMode(digitalPins[i], INPUT);
        pinMode(analogPins[i], INPUT);
    }
}

void Sensors::readSensorsAnalog()
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