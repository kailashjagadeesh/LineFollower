#include <QTRSensors.h>
#include "QTRSensorsAnalog.h"

QTRSensorsAnalog::QTRSensorsAnalog(const uint8_t* pins, uint8_t numSensors, uint16_t timeout = TIMEOUT):QTRSensors() {
    setTypeAnalog();
    setSensorPins(pins, numSensors);
    //setEmitterPin(emitterPin);
    //setTimeout(timeout);
}

uint16_t  QTRSensorsAnalog::readLine(uint16_t* sensorValues) {
#ifdef LF_BLACKLINE_LOGIC
    return readLineBlack(sensorValues);
#else
    return readLineWhite(sensorValues);
#endif
}
