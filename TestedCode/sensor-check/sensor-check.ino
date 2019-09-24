#include "QTRSensorsAnalog.h"
#include "pid_control.h"
#include "motor_control.h"
#include <SoftwareSerial.h>

#define PID_IDEAL 3500
#define baseSpeed 255
#define SENSOR_PINS \
    (const uint8_t[]) { A0, A1, A2, A3, A4, A5, A6, A7 }
#define NUM_SENSORS 8

SoftwareSerial bluetooth(12, 11);
#define MOTOR_RIGHT_PINS \
    (const uint8_t[]) { 7, 8, 10 }
#define MOTOR_LEFT_PINS \
    (const uint8_t[]) { 5, 6, 9 }
#define MOTOR_STANDBY_PIN \
    (const uint8_t)4

QTRSensorsAnalog qtr(SENSOR_PINS, NUM_SENSORS);
uint16_t sensorValues[NUM_SENSORS];
uint16_t line;
PIDControl pid(PID_IDEAL);
Motor motor(MOTOR_LEFT_PINS, MOTOR_RIGHT_PINS, MOTOR_STANDBY_PIN);

void PID_tune() // Auto tune implemented using twiddle algorithm
{
    bluetooth.println("PID tuning...");
    float best_err = abs(pid.getError(qtr.readLine(sensorValues)));
    float err;

    double sum = (pid.dp[0] + pid.dp[1] + pid.dp[2]);

    while (sum > 0.001)
    {
        for (int i = 0; i < 2; i++)
        {
            pid.parameters[i] += pid.dp[i];
            err = follow();
            if (err < best_err)
            {
                best_err = err;
                pid.dp[i] *= 1.1;
            }
            else
            {
                pid.parameters[i] -= 2 * pid.dp[i];
                err = follow();

                if (err < best_err)
                {
                    best_err = err;
                    pid.dp[i] *= 1.1;
                }
                else
                {
                    pid.parameters[i] += pid.dp[i];
                    pid.dp[i] *= 0.9;
                }
            }
        }
        sum = (pid.dp[0] + pid.dp[1] + pid.dp[2]);
    }

    bluetooth.println("Done PID tuning");
    bluetooth.println("kp: ");
    bluetooth.println(*pid.kp);
    bluetooth.println("kd: ");
    bluetooth.println(*pid.kd);
    bluetooth.println("ki: ");
    bluetooth.println(*pid.ki);
}

void setup()
{
    bluetooth.begin(9600);

    pinMode(2, OUTPUT);

    //callibration
    digitalWrite(2, HIGH);
    bluetooth.println("Calibrating");
    for (int i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }
    bluetooth.println("Done calib");
    //PID_tune();
    digitalWrite(2, LOW);
}
unsigned int follow()
{
    line = qtr.readLine(sensorValues);
    int16_t correction = pid.control((int)line);

    bluetooth.println(correction);
    int16_t newSpeed = baseSpeed;
    newSpeed -= abs(correction);
    if (newSpeed < 0)
        newSpeed = 0;

    if (correction > 0)
    {
        motor.setLeftSpeed(baseSpeed);
        motor.setRightSpeed(newSpeed);
    }
    else
    {
        motor.setLeftSpeed(newSpeed);
        motor.setRightSpeed(baseSpeed);
    }

    motor.setLeftDirection(Motor::Front);
    motor.setRightDirection(Motor::Front);

    return abs(pid.getError(qtr.readLine(sensorValues)));
}
void loop()
{
    follow();
}