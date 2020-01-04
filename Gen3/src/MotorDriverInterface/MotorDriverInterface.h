#ifndef MOTOR_DRIVER_INTERFACE_H
#define MOTOR_DRIVER_INTERFACE_H

#include <stdint.h>

//change A with B to swap left and right motors
#define LEFTMOTOR_B

class Motor {
    //pin definitions
    typedef int pin;

    const pin STBY = 13;
    const pin PWMA = 12;
    const pin AIN2 = 11;
    const pin AIN1 = 10;
    const pin BIN1 = 9;
    const pin BIN2 = 8;
    const pin PWMB = 7;

    bool enabled;

#ifdef LEFTMOTOR_A
    const pin &leftFront = AIN1;
    const pin &rightFront = BIN1;

    const pin &leftBack = AIN2;
    const pin &rightBack = BIN2;

    const pin &leftSpeed = PWMA;
    const pin &rightSpeed = PWMB;
#else
    const pin &leftFront = BIN1;
    const pin &rightFront = AIN1;

    const pin &leftBack = BIN2;
    const pin &rightBack = AIN2;

    const pin &leftSpeed = PWMB;
    const pin &rightSpeed = PWMA;
#endif

public:
    enum Direction {FRONT, BACK};

    Motor();
    void stopMotors();
    void setLeftSpeed(uint8_t);
    void setRightSpeed(uint8_t);
    void setLeftDirection(Direction);
    void setRightDirection(Direction);
    void enable();
    void disable();
};

#endif