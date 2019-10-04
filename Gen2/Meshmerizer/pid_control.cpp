#include "pid_control.h"

#define convFactor 100.0f  // Factor for scaling O/P from sensor to feed PID algo

PIDControl::PIDControl(uint16_t _targetValue)
{
    targetValue = _targetValue/convFactor;

    errorSum = 0;
    lastError = 0;

    kp = parameters;
    kd = parameters + 1;
    ki = parameters + 2;

    parameters[0] = 5.0;  // Hard coded PID values
    parameters[1] = 4.35;
    parameters[2] = 0.00;

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
    float error = currentValue/convFactor - targetValue;
    //errorSum += error;
    errorSum = error + lastError;

    c = (*kp) * error + (*kd) * (error - lastError) + (*ki) * errorSum;
    lastError = error;
    return c;
}
float PIDControl::getError(int16_t currentValue)
{
    return currentValue/convFactor - targetValue;
}
void PIDControl::clear()
{
    errorSum = 0;
    lastError = 0;
}