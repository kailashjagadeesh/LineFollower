#include <QTRSensors.h>

#ifndef QTRSENSORSANALOG_H
#define QTRSENSORSANALOG_H

class QTRSensorsAnalog : public QTRSensors{
    public:

    QTRSensorsAnalog(const uint8_t* pins, uint8_t numSensors);
    uint16_t readLine(uint16_t *sensorValues);
};

#endif