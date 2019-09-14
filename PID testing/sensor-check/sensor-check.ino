#include "QTRSensorsAnalog.h"

#define SENSOR_PINS (const uint8_t[]) {A0, A1, A2, A3, A4, A5, A6, A7}
#define NUM_SENSORS 8

QTRSensorsAnalog qtr(SENSOR_PINS, NUM_SENSORS);
uint16_t sensorValues[NUM_SENSORS];

void setup() {
    Serial.begin(9600);

    pinMode(13, OUTPUT);

    //callibration
    pinMode(13, HIGH);
    for (int i = 0; i < 400; i++) {
        qtr.calibrate();
    }
    pinMode(13, LOW);
}

void loop() {
    uint16_t line = qtr.readLine(sensorValues);
    Serial.println(line);
    delay(1000);
}