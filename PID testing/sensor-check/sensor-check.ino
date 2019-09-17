#include "QTRSensorsAnalog.h"
#include "pid_control.h"
#include "motor_control.h"
#include <SoftwareSerial.h>

#define SENSOR_PINS \
    (const uint8_t[]) { A0, A1, A2, A3, A4, A5, A6, A7 }
#define NUM_SENSORS 8

#define KP 0.01
#define KD 0
#define KI 0

SoftwareSerial bluetooth(12, 11);
#define MOTOR_RIGHT_PINS \
    (const uint8_t[]) { 7, 8, 10 }
#define MOTOR_LEFT_PINS \
    (const uint8_t[]) { 5, 6, 9 }

QTRSensorsAnalog qtr(SENSOR_PINS, NUM_SENSORS);
uint16_t sensorValues[NUM_SENSORS];
uint16_t line;
PIDControl pid(line, 3500, KP, KD, KI);
Motor motor(MOTOR_LEFT_PINS, MOTOR_RIGHT_PINS);

void setup()
{
    bluetooth.begin(9600);
    Serial.begin(115200);

    pinMode(13, OUTPUT);

    //callibration
    digitalWrite(13, HIGH);
    Serial.println("Calibrating");
    bluetooth.println("Calibrating");
    for (int i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }
    Serial.println("Done calib");
    bluetooth.println("Done calib");
    digitalWrite(13, LOW);
}

void loop()
{
    line = qtr.readLine(sensorValues);

    int16_t correction = pid.control();
    if (correction > 0)
    {
        motor.setLeftSpeed(1024);
        motor.setRightSpeed(1024 - correction);
    }
    else
    {
        motor.setLeftSpeed(1024 + correction);
        motor.setRightSpeed(1024);
    }
    motor.setLeftDirection(Motor::Front);
    motor.setRightDirection(Motor::Front);
    Serial.println(correction);
    bluetooth.println(correction);

    delay(200);
}