#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include "TesterInterface.h"
#include "LEDInterface.h"

#ifndef NUM_SENSORS
#define NUM_SENSORS 9
#endif

#ifndef NUM_PIDSENSORS
#define NUM_PIDSENSORS 5
#endif

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

    //digital pin outputs
    uint16_t digitalReadings[12];

    //analog outputs converted to digital using calibrated threshold values
    uint16_t digitalValues;

    //read values
    //read analog values and fill analogReadings[]
    void readSensorsAnalog();
    //read digital pins and fill digitalReadings[] 
    void readSensorsDigital(volatile uint8_t *, volatile bool *);
    //read both analog and digital pins
    void readSensors();
    //convert analogReadings[] to digital form and fill digitalValues
    void convertAnalogToDigital();

    //callibration of min, max and avg values of each sensor and hence threshold values
    void calibrate();

    //get line for PID
    uint16_t readLine();

    Sensors();

    //debug info
    //print calibrated data
    void printCalibratedInfo();
    //print analogReadings
    void printAnalogReadings();
    //print digitalReadings
    void printDigitalReadings();
    //print digitalValues
    void printDigitalValues();
};

//pin definitions
const uint8_t Sensors::analogPins[12] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
const uint8_t Sensors::digitalPins[12] = {6, 5, 4, 3, 36, 28, 38, 26, 32, 34, 30, 40};

void Sensors::printDigitalValues()
{
    //SERIALD.print("Converted values: ");
    //SERIALD.println(digitalValues, BIN);
}

// void Sensors::attachAllInterrupts()
// {
//     attachInterrupt(digitalPinToInterrupt(CFPin), CFISR, CHANGE);

//     attachInterrupt(digitalPinToInterrupt(LFPin), LFISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(LMPin), LMISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(LB1Pin), LB1ISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(LB2Pin), LB2ISR, CHANGE);

//     attachInterrupt(digitalPinToInterrupt(RFPin), RFISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(RMPin), RMISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(RB1Pin), RB1ISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(RB2Pin), RB2ISR, CHANGE);
// }

void Sensors::printCalibratedInfo()
{
    //SERIALD.println("Calibrated values: \n");
    //SERIALD.println("WHITE:");
    //SERIALD.print("MAX VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        //SERIALD.print(whiteValues.highValues[i]);
        //SERIALD.print("\t");
    }
    //SERIALD.print("\nMIN VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        //SERIALD.print(whiteValues.lowValues[i]);
        //SERIALD.print("\t");
    }
    //SERIALD.print("\nAVG VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        //SERIALD.print(whiteValues.averageValues[i]);
        //SERIALD.print("\t");
    }

    //SERIALD.println("\n\n\nBLACK:");
    //SERIALD.print("MAX VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        //SERIALD.print(blackValues.highValues[i]);
        //SERIALD.print("\t");
    }
    //SERIALD.print("\nMIN VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        //SERIALD.print(blackValues.lowValues[i]);
        //SERIALD.print("\t");
    }
    //SERIALD.print("\nAVG VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        //SERIALD.print(blackValues.averageValues[i]);
        //SERIALD.print("\t");
    }

    //SERIALD.println("\n\n\nTHRESHOLD VALUES:");
    //SERIALD.print("THRESHOLD VALUES:\t");
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        //SERIALD.print(calibratedValues.thresholdValues[i]);
        //SERIALD.print("\t");
    }
}

void Sensors::printAnalogReadings()
{
    //SERIALD.print("\n\nAnalog: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        //SERIALD.print(analogReadings[i]);
        //SERIALD.print("\t");
    }
}

void Sensors::printDigitalReadings()
{

    //SERIALD.print("\n\nDigital: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        //SERIALD.print(digitalReadings[i]);
        //SERIALD.print("\t");
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

    LED::write(0, HIGH);
    LED::write(1, HIGH);

    //SERIALD.println("Keep on white surface");
    delay(5000);
    //SERIALD.println("reading values...");
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

    LED::write(0, LOW);
    LED::write(1, LOW);

    //SERIALD.println("Keep on black surface");
    delay(5000);
    //SERIALD.println("reading values...");
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
        if (analogReadings[i] > calibratedValues.thresholdValues[i])
        {
            digitalValues |= (1 << (NUM_SENSORS - i - 1));
        }
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

#endif