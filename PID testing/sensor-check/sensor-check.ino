#include "QTRSensorsAnalog.h"
#include "pid_control.h"
#include "motor_control.h"
#include <SoftwareSerial.h>

#define PID_IDEAL 3500
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
#define MOTOR_STANDBY_PIN \
    (const uint8_t) 4

QTRSensorsAnalog qtr(SENSOR_PINS, NUM_SENSORS);
uint16_t sensorValues[NUM_SENSORS];
uint16_t line;
PIDControl pid(PID_IDEAL, KP, KD, KI);
Motor motor(MOTOR_LEFT_PINS, MOTOR_RIGHT_PINS,MOTOR_STANDBY_PIN);

void setup()
{
    bluetooth.begin(9600);
    Serial.begin(115200);

    pinMode(2, OUTPUT);

    //callibration
    digitalWrite(2, HIGH);
    Serial.println("Calibrating");
    bluetooth.println("Calibrating");
    for (int i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }
    Serial.println("Done calib");
    bluetooth.println("Done calib");
    digitalWrite(2, LOW);
}

void loop()
{
    line = qtr.readLine(sensorValues);
    //bluetooth.println(line);
    int16_t correction = pid.control((int)line);
    if (correction > 0)
    {
        motor.setLeftSpeed(122);
        motor.setRightSpeed(122 - correction);
    }
    else
    {
        motor.setLeftSpeed(122 + correction);
        motor.setRightSpeed(122);
    }
    motor.setLeftDirection(Motor::Front);
    motor.setRightDirection(Motor::Front);
    Serial.println(correction);
    bluetooth.println(correction);

    delay(200);
}