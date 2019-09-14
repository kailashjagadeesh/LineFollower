#include <QTRSensors.h>
#include "QTRSensorsAnalog.h"

QTRSensorsAnalog::QTRSensorsAnalog(const uint8_t* pins, uint8_t numSensors):QTRSensors() {
    setTypeAnalog();
    setSensorPins(pins, numSensors);
    //setTimeout(timeout);
}

uint16_t  QTRSensorsAnalog::readLine(uint16_t* sensorValues) {
#ifndef LF_BLACKLINE_LOGIC
    return readLineBlack(sensorValues);
#else
    return readLineWhite(sensorValues);
#endif
}