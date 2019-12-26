#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include "../LEDInterface/LEDInterface.h"
#include "../PushButtonInterface/PushButtonInterface.h"
#include "../BluetoothInterface/BluetoothInterface.h"
#include <stdint.h>


class Sensors
{
    //ds to strore the readings
    uint16_t analogReadings[12];

    //pin definitions
    static const uint8_t analogPins[12];
    static const uint8_t digitalPins[12];
    static const uint8_t sensorCenterPins[12];

    Bluetooth bluetooth;

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
        uint16_t highValues[12];
        uint16_t lowValues[12];
        uint16_t averageValues[12];
    } whiteValues, blackValues;

    struct
    {
        uint16_t thresholdValues[12];
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


#endif