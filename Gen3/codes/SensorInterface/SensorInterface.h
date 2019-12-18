#include "ISRs.h"
#include "LEDInterface.h"
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
    struct CalibratedValues
    {
        uint16_t highValues[NUM_SENSORS];
        uint16_t lowValues[NUM_SENSORS];
        uint16_t averageValues[NUM_SENSORS];
    } whiteValues, blackValues;

    struct
    {
        uint16_t thresholdValues[NUM_SENSORS];
    } calibratedValues;

    uint16_t digitalReadings[12];
    uint16_t digitalValues;
    void attachAllInterrupts();
    //read values
    void readSensorsAnalog();
    void readSensorsDigital(volatile uint8_t *, volatile bool *);
    void readSensors();
    void convertAnalogToDigital();

    //callibration (set DAC Values)
    void calibrate();

    //get line for PID
    uint16_t readLine();

    Sensors();

    //debug info
    void printCalibratedInfo();
    void printAnalogReadings();
    void printDigitalReadings();
    void printDigitalValues();
};

//pin definitions
const uint8_t Sensors::analogPins[12] = {A0, A1, A2, A3, A11, A5, A9, A7, A8, A10, A6, A4};
const uint8_t Sensors::digitalPins[12] = {6, 5, 4, 3, 36, 28, 38, 26, 32, 34, 30, 40};

void Sensors::printDigitalValues()
{
    Serial.print("Converted values: ");
    Serial.println(digitalValues, BIN);
}

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
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        Serial.print(whiteValues.highValues[i]);
        Serial.print("\t");
    }
    Serial.print("\nMIN VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        Serial.print(whiteValues.lowValues[i]);
        Serial.print("\t");
    }
    Serial.print("\nAVG VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        Serial.print(whiteValues.averageValues[i]);
        Serial.print("\t");
    }

    Serial.println("\n\n\nBLACK:");
    Serial.print("MAX VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        Serial.print(blackValues.highValues[i]);
        Serial.print("\t");
    }
    Serial.print("\nMIN VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        Serial.print(blackValues.lowValues[i]);
        Serial.print("\t");
    }
    Serial.print("\nAVG VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        Serial.print(blackValues.averageValues[i]);
        Serial.print("\t");
    }

    Serial.println("\n\n\nTHRESHOLD VALUES:");
    Serial.print("THRESHOLD VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        Serial.print(calibratedValues.thresholdValues[i]);
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

void Sensors::calibrate()
{
    //clear the readings
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        whiteValues.highValues[i] = whiteValues.lowValues[i] = 0;
        blackValues.highValues[i] = blackValues.lowValues[i] = 0;
        calibratedValues.thresholdValues[i] = 0;
    }

    led.write(LED::one, HIGH);
    led.write(LED::two, HIGH);

    Serial.println("Keep on white surface");
    delay(5000);
    Serial.println("reading values...");
    for (int i = 0; i < 100; i++)
    {
        readSensorsAnalog();
        for (int j = 0; j < NUM_SENSORS; ++j)
        {
            if (analogReadings[j] > whiteValues.highValues[j])
                whiteValues.highValues[j] = analogReadings[j];

            if (i == 0 || analogReadings[j] < whiteValues.lowValues[j])
                whiteValues.lowValues[j] = analogReadings[j];
        }
    }

    led.write(LED::one, LOW);
    led.write(LED::two, LOW);

    Serial.println("Keep on black surface");
    delay(5000);
    Serial.println("reading values...");
    for (int i = 0; i < 100; i++)
    {
        readSensorsAnalog();
        for (int j = 0; j < NUM_SENSORS; ++j)
        {
            if (analogReadings[j] > blackValues.highValues[j])
                blackValues.highValues[j] = analogReadings[j];

            if (i == 0 || analogReadings[j] < blackValues.lowValues[j])
                blackValues.lowValues[j] = analogReadings[j];
        }
    }

    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        whiteValues.averageValues[i] = (whiteValues.highValues[i] + whiteValues.lowValues[i]) / 2;
        blackValues.averageValues[i] = (blackValues.highValues[i] + blackValues.lowValues[i]) / 2;
        calibratedValues.thresholdValues[i] = (whiteValues.lowValues[i] + blackValues.highValues[i]) / 2;
    }
}

void Sensors::convertAnalogToDigital()
{
    digitalValues = 0;
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
#ifdef WHITELINE_LOGIC
        if (analogReadings[i] > calibratedValues.thresholdValues[i])
        {
            digitalValues |= (1 << (NUM_SENSORS - i - 1));
        }
#else
        if (analogReadings[i] < calibratedValues.thresholdValues[i])
        {
            digitalValues |= (1 << (NUM_SENSORS - i - 1));
        }
#endif
    }
}

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
#ifdef WHITELINE_LOGIC
        analogReadings[i] = analogRead(analogPins[i]);
#else
        analogReadings[i] = 1023 - analogRead(analogPins[i]);
#endif
    }
}
// CF pin must be indexed 8th(count from 0)
void Sensors::readSensorsDigital(volatile uint8_t *backSensorState, volatile bool *CFState)
{
    *backSensorState = 0;
    *CFState = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
#ifdef WHITELINE_LOGIC
        digitalReadings[i] = digitalRead(digitalPins[i]);
#else
        digitalReadings[i] = !digitalRead(digitalPins[i]);
#endif
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

//Reads the sensor analog values and returns the position of the line
uint16_t Sensors::readLine()
{
    uint16_t sum[2] = {0, 0};
    readSensorsAnalog();
    convertAnalogToDigital();
    for (int i = 2; i < NUM_SENSORS - 4; ++i)
    {
        if (digitalValues & (NUM_SENSORS - i - 1))
        {
            sum[0] += (i - 2) * 1000;
            sum[1] += 1;
        }
    }

    return (sum[1] > 0) ? (sum[0] / sum[1]) : 2000;
}

#endif