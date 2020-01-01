#include "PIDControl.h"

#include <Arduino.h>
#include "../MotorDriverInterface/MotorDriverInterface.h"
#include "../LEDInterface/LEDInterface.h"
#include "../configure.h"
#include "../TesterInterface/TesterInterface.h"

#ifndef KP
#define KP 1
#endif

#ifndef KD
#define KD 1
#endif

#ifndef KI
#define KI 1
#endif

#ifndef KP2
#define KP2 1
#endif

#ifndef KD2
#define KD2 1
#endif

#ifndef KI2
#define KI2 1
#endif

#ifndef PID_CONVFACTOR
#define PID_CONVFACTOR 1
#endif

#ifndef PID_THRESHOLD
#define PID_THRESHOLD 1
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
    parameters[3] = KP2;
    parameters[4] = KD2;
    parameters[5] = KI2;

    tuning = true;

    dp[0] = 3;
    dp[1] = 4;
    dp[2] = 0.00;

    increaseTime = 0;
}

int16_t PIDControl::control(int16_t currentValue)
{
    int16_t c; //correction
    float error = currentValue / PID_CONVFACTOR - targetValue;
    errorSum = error + lastError;
    lastError = error;


    // if (abs(error) <= PID_THRESHOLD)
    float P = (*kp) * error;
    float D =(*kd) * (error - lastError);
    float I = (*ki) * errorSum;
    c = P + D + I;  
        // if (abs(error) >= PID_THRESHOLD)
        //     errorSum = 0;  
    // else {
    //     c = (*(kp+3)) * error + (*(kd+3)) * (error - lastError) + (*(ki+3)) * errorSum;
    //     Debug::print("-");
    //     errorSum = 0;
    // }  

    if (millis() < increaseTime) {
        c = (*(kp+3)) * error + (*(kd+3)) * (error - lastError) + (*(ki+3)) * errorSum;   
    }
    return c;
}

float PIDControl::getError(int16_t currentValue)
{
    return currentValue / PID_CONVFACTOR - targetValue;
}

void PIDControl::increasePID(uint32_t m) {
    increaseTime = m + millis();
}

void PIDControl::clear()
{
    errorSum = 0;
    lastError = 0;
}

void MotorPIDControl::setBaseSpeed(uint8_t speed)  {
    baseSpeed = speed;
    errorSum = 0;
}


void MotorPIDControl::setSpeedBasedOnCorrection(int16_t correction) {
    if (baseSpeed < 250) {
        baseSpeed += 5;
    }
    else {
        baseSpeed = 255;
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

