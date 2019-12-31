#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <stdint.h>
#include "../LEDInterface/LEDInterface.h"

class Sensors
{
    ///ds to strore the readings
    uint16_t analogReadings[12];

    //pin definitions
    static const uint8_t analogPins[12];
    static const uint8_t digitalPins[12];
    static const uint8_t sensorCenterPins[12];
    static const uint8_t centerRearPin;

    struct {
        int overshoots[3];
        int back = -1;
        int front = -1;

        int left = 0;
        int right = 1;

        uint8_t prevState = 0;

        int nextOvershoot() {
            return overshoots[front];
        }

        void push(int d) {
            back ++;
            overshoots[back] = d;
            if (front == -1)    
                front = 0;
        }

        void pop() {
            if (front != -1)
                overshoots[front] = -1;
            if (front == back) {
                front = back = -1;
            }
            else 
                front++;
        }

        bool empty() {
            return (front == -1);
        }
    } overshootData;


public:
    uint8_t rearSensorStatus;

    struct CalibratedValues
    {
        uint16_t highValues[12];
        uint16_t lowValues[12];
        uint16_t averageValues[12];
    } whiteValues, blackValues;

    struct
    {
        uint16_t thresholdValues[12];
    } calibratedValues;

    bool onBoardCenterPin;
   
   //digital pin outputs
    uint16_t digitalReadings[12];
    //analog outputs converted to digital using calibrated threshold values
    uint16_t digitalValues;
    uint8_t nSensorsOnLine;

    ///read values
    //read analog values and fill analogReadings[]
    void readSensorsAnalog();
    //read digital pins and fill digitalReadings[] 
    void readSensorsDigital();
    void readSensors();
    void readCenterSensors();
    uint8_t readRearSensors();
    bool rearCenterStatus();
    //convert analogReadings[] to digital form and fill digitalValues
    void convertAnalogToDigital();

    //callibration of min, max and avg values of each sensor and hence threshold values
    void calibrate();

    //get line for PID
    uint16_t readLine();

    //construction
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
    void updateAllSensors();
    void printAllSensors();

    ///overshoot interface
    bool overshootCompleted();
    void addLeftOvershoot();
    void addRightOvershoot();
    void waitForOvershoot();
    void overshootControl(bool debug = false);

};

#endif