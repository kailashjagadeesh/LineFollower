#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#ifndef PID_CONVFACTOR
#define PID_CONVFACTOR 1
#endif

#include "MotorDriverInterface.h"

class PIDControl
{
    float targetValue;
    float errorSum, lastError;

public:
    double *kp, *kd, *ki;
    double parameters[6], dp[3];
    PIDControl(uint16_t _targetValue);
    float getError(int16_t currentValue);
    int16_t control(int16_t currentValue);
    void clear();
};

PIDControl::PIDControl(uint16_t _targetValue)
{
    targetValue = _targetValue / PID_CONVFACTOR;

    errorSum = 0;
    lastError = 0;

    kp = parameters;
    kd = parameters + 1;
    ki = parameters + 2;

    parameters[0] = 5.1; // Hard coded PID values
    parameters[1] = 4.35;
    parameters[2] = 0.00;

    //for sharp turns
    parameters[3] = 8;
    parameters[4] = 4.35;
    parameters[5] = 0;

    /*
        Best values so far 
        5.00
        4.35
        0.00

        4.85
        4.35
        0.01

        3.64
        4.35
        0.00

        4.75
        5.06
        0.00
     */

    dp[0] = 1.00;
    dp[1] = 1.00;
    dp[2] = 0.00;
}

int16_t PIDControl::control(int16_t currentValue)
{
    int16_t c; //correction
    float error = currentValue / PID_CONVFACTOR - targetValue;
    //errorSum += error;
    errorSum = error + lastError;

    if (abs(error) > 1000/PID_CONVFACTOR) {
        c = (*(kp+3)) * error + (*(kd+3)) * (error - lastError) + (*(ki+3)) * errorSum;
    }
    else {
        c = (*kp) * error + (*kd) * (error - lastError) + (*ki) * errorSum;    
    }

    lastError = error;
    return c;
}

float PIDControl::getError(int16_t currentValue)
{
    return currentValue / PID_CONVFACTOR - targetValue;
}

void PIDControl::clear()
{
    errorSum = 0;
    lastError = 0;
}

class MotorPIDControl : public PIDControl
{
    uint8_t baseSpeed;
    Motor &motors;
public:
    MotorPIDControl(uint16_t _targetValue, uint8_t _baseSpeed, Motor& _motors) 
        : PIDControl(_targetValue), motors(_motors) {
        baseSpeed = _baseSpeed;
    }

    void setSpeedBasedOnCorrection(int16_t correction);
};

void MotorPIDControl::setSpeedBasedOnCorrection(int16_t correction) {
    uint8_t newSpeed = baseSpeed;
    newSpeed -= abs(correction);

    if (correction > 0) {
        motors.setRightSpeed(newSpeed);
        motors.setLeftSpeed(baseSpeed);
    }
    else {
        motors.setRightSpeed(baseSpeed);
        motors.setLeftSpeed(newSpeed);
    }
}

#endif