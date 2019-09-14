#include <QTRSensors.h>

#ifndef QTRSENSORSANALOG_H
#define QTRSENSORSANALOG_H

#define TIMEOUT 4                      // waits for 2500 us for sensor outputs to go low
class QTRSensorsAnalog : public QTRSensors{
    public:

    QTRSensorsAnalog(const uint8_t* pins, uint8_t numSensors ,uint16_t timeout = TIMEOUT);
    uint16_t readLine(uint16_t *sensorValues);
};

#endif
