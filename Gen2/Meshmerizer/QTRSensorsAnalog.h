#include <QTRSensors.h>

#ifndef QTRSENSORSANALOG_H
#define QTRSENSORSANALOG_H
/*
Custom class for using qtr sensor. Inherited all features of default class. 
*/
class QTRSensorsAnalog : public QTRSensors
{
public:
    QTRSensorsAnalog(const uint8_t *pins, uint8_t numSensors);
    uint16_t readLine(uint16_t *sensorValues);
    void generateThreshold(uint16_t *threshold, uint16_t *);
};
 // Initialize sensor to analog type and pass the number of pins used
QTRSensorsAnalog::QTRSensorsAnalog(const uint8_t *pins, uint8_t numSensors) : QTRSensors()
{
    setTypeAnalog();
    setSensorPins(pins, numSensors);
    //setTimeout(timeout);
}
/*
Sets thresholds for determining if detected threshold is black/white 
*/
void QTRSensorsAnalog::generateThreshold(uint16_t *thresholdOff, uint16_t *thresholdOn) // Threshold is set to average of min and max
{
    for (int i = 0; i < 8; i++)
    {
        thresholdOff[i] = (calibrationOff.maximum[i] + calibrationOff.minimum[i]) / 2;
    }

    for (int i = 0; i < 8; i++)
    {
        thresholdOn[i] = (calibrationOn.maximum[i] + calibrationOn.minimum[i]) / 2;
    }
}
/*
 This function reads all sensors and returns the weighted sensor readings.

 Used for both black and whiteline logics 
*/
uint16_t QTRSensorsAnalog::readLine(uint16_t *sensorValues)
{
#ifdef LF_BLACKLINE_LOGIC
    return readLineBlack(sensorValues);
#else
    return readLineWhite(sensorValues);
#endif
}

#endif