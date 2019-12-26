#include "PIDControl.h"

#include <Arduino.h>
#include "../MotorDriverInterface/MotorDriverInterface.h"
#include "../LEDInterface/LEDInterface.h"
#include "../configure.h"

#ifndef KP
#define KP 1
#endif

#ifndef KD
#define KD 1
#endif

#ifndef KI
#define KI 1
#endif

#ifndef PID_CONVFACTOR
#define PID_CONVFACTOR 1
#endif

PIDControl::PIDControl(uint16_t _targetValue)
{
    targetValue = _targetValue / PID_CONVFACTOR;

    errorSum = 0;
    lastError = 0;

    kp = parameters;
    kd = parameters + 1;
    ki = parameters + 2;

    parameters[0] = KP; // Hard coded PID values
    parameters[1] = KD;
    parameters[2] = KI;

    //for sharp turns
    parameters[3] = parameters[0];
    parameters[4] = parameters[1];
    parameters[5] = 0;

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
        LED::write(0, HIGH);
        c = (*(kp+3)) * error + (*(kd+3)) * (error - lastError) + (*(ki+3)) * errorSum;
    }
    else {
        LED::write(0, LOW);
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

void MotorPIDControl::setBaseSpeed(uint8_t speed)  {
    baseSpeed = speed;
}


void MotorPIDControl::setSpeedBasedOnCorrection(int16_t correction) {
    if (baseSpeed < 255) {
        baseSpeed ++;
    }

    uint8_t newSpeed = baseSpeed;
    newSpeed -= (abs(correction) > baseSpeed) ? baseSpeed : abs(correction);
    
    if (correction > 0) {
        motors.setRightSpeed(newSpeed);
        motors.setLeftSpeed(baseSpeed);
    }
    else {
        motors.setRightSpeed(baseSpeed);
        motors.setLeftSpeed(newSpeed);
    }
}