#include "MotorDriverInterface.h"
#include <Arduino.h>


void Motor::disable() {
    enabled = false;
    stopMotors();
}

void Motor::enable() {
    enabled = true;
}

Motor::Motor() {
    pinMode(leftFront, OUTPUT);
    pinMode(leftBack, OUTPUT);
    pinMode(leftSpeed, OUTPUT);

    pinMode(rightFront, OUTPUT);
    pinMode(rightBack, OUTPUT);
    pinMode(rightSpeed, OUTPUT);

    pinMode(STBY, OUTPUT);
    enabled = true;

    //the motors don't work otherwise
    stopMotors();
}

//stops the motors
void Motor::stopMotors() {
    digitalWrite(STBY,LOW);

    setLeftSpeed(0);
    setRightSpeed(0);

    digitalWrite(STBY,HIGH);
}

//setLeftSpeed(speed: the speed to be set : 0 <= speed <= 255)
void Motor::setLeftSpeed(uint8_t speed)
{
    // unit8_t speedCorrection = 0;
    if (enabled)
        // if  (speed > 55) analogWrite(leftSpeed, speed - 55);
        // else analogWrite(leftSpeed, 0);
        analogWrite(leftSpeed, speed);
}

//setRightSpeed(speed: the speed to be set : 0 <= speed <= 255)
void Motor::setRightSpeed(uint8_t speed)
{
    if (enabled)
        analogWrite(rightSpeed, speed);
}

void Motor::setLeftDirection(Direction dir)
{
    if (dir == FRONT)
    {
        digitalWrite(leftFront, 1);
        digitalWrite(leftBack, 0);
    }
    else
    {
        digitalWrite(leftFront, 0);
        digitalWrite(leftBack, 1);
    }
}

void Motor::setRightDirection(Direction dir)
{
    if (dir == FRONT)
    {
        digitalWrite(rightFront, 1);
        digitalWrite(rightBack, 0);
    }
    else
    {
        digitalWrite(rightFront, 0);
        digitalWrite(rightBack, 1);
    }
}