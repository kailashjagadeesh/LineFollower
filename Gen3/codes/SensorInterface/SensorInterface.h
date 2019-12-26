#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include "TesterInterface.h"
#include "LEDInterface.h"
#include "PushButtonInterface.h"

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
    static const uint8_t sensorCenterPins[NUM_SENSOR_CENTER_PINS];

    private:
    struct {
        int overshoots[2];
        int index = -1;

        int left = 0;
        int right = 1;

        uint8_t prevState = 0;

        int nextOvershoot() {
            return overshoots[0];
        }

        void push(int d) {
            index++;
            overshoots[index] = d;
        }

        void pop() {
            if (index >= 0) {
                index --;
                overshoots[index] = overshoots[index+1];
                overshoots[index+1] = 0;
            }
        }
    } overshootData;


public:

    uint8_t rearSensorStatus;

    struct CalibratedValues
    {
        uint16_t highValues[NUM_SENSORS+2];
        uint16_t lowValues[NUM_SENSORS+2];
        uint16_t averageValues[NUM_SENSORS+2];
    } whiteValues, blackValues;

    struct
    {
        uint16_t thresholdValues[NUM_SENSORS+2];
    } calibratedValues;

    //digital pin outputs
    uint16_t digitalReadings[12];

    //analog outputs converted to digital using calibrated threshold values
    uint16_t digitalValues;
    uint8_t nSensorsOnLine;

    //read values
    //read analog values and fill analogReadings[]
    void readSensorsAnalog();
    //read digital pins and fill digitalReadings[] 
    void readSensorsDigital(volatile uint8_t *, volatile bool *);
    //read both analog and digital pins
    void readSensors();
    void readCenterSensors();
    uint8_t readRearSensors();
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

    ///overshoot interface
    bool overshootCompleted();
    void addLeftOvershoot();
    void addRightOvershoot();
    void waitForOvershoot();
    void overshootControl();

};

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

    if (((overshootData.nextOvershoot() == overshootData.left) && (prevLeft - currentLeft > 0))
        || ((overshootData.nextOvershoot() == overshootData.right) && (prevRight - currentRight > 0))) {
        overshootData.pop();
    } 

    overshootData.prevState = currentState;
}

//pin definitions
const uint8_t Sensors::analogPins[12] = {A8, A7, A6, A5, A4, A3, A2, A1, A0, A9, A10, A11};
const uint8_t Sensors::digitalPins[12] = {6, 5, 4, 3, 36, 28, 38, 26, 32, 34, 30, 40};
const uint8_t Sensors::sensorCenterPins[NUM_SENSOR_CENTER_PINS] = SENSOR_CENTER_PINS;

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
    SERIALD.print("Converted values: ");
    SERIALD.println(digitalValues, BIN);
}

void Sensors::printCalibratedInfo()
{
    SERIALD.println("Calibrated values: \n");
    SERIALD.println("WHITE:");
    SERIALD.print("MAX VALUES:\n\t");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        SERIALD.print(whiteValues.highValues[i]);
        SERIALD.print("\t");
    }
    SERIALD.print("\nMIN VALUES:\n\t");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        SERIALD.print(whiteValues.lowValues[i]);
        SERIALD.print("\t");
    }

    SERIALD.println("\n\n\nBLACK:");
    SERIALD.print("MAX VALUES:\n\t");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        SERIALD.print(blackValues.highValues[i]);
        SERIALD.print("\t");
    }
    SERIALD.print("\nMIN VALUES:\n\t");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        SERIALD.print(blackValues.lowValues[i]);
        SERIALD.print("\t");
    }

    SERIALD.println("\n\nTHRESHOLD VALUES:\n\t");
    SERIALD.print("THRESHOLD VALUES:\n");
    for (int i = 0; i < NUM_SENSORS + 2; ++i)
    {
        SERIALD.print(calibratedValues.thresholdValues[i]);
        SERIALD.print("\t");
    }
}

void Sensors::printAnalogReadings()
{
    SERIALD.print("\n\nAnalog: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        SERIALD.print(analogReadings[i]);
        SERIALD.print("\t");
    }
}

void Sensors::printDigitalReadings()
{

    SERIALD.print("\n\nDigital: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        SERIALD.print(digitalReadings[i]);
        SERIALD.print("\t");
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

    SERIALD.println("Keep on white surface");
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

    SERIALD.println("Keep on black surface");
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