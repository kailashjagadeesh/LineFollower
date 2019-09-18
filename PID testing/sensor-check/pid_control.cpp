#include "pid_control.h"

PIDControl::PIDControl(uint16_t _targetValue)
{
    targetValue = _targetValue;

    errorSum = 0;
    lastError = 0;

    kp = &parameters[0];
    kd = &parameters[1];
    ki = &parameters[2];

    parameters[0] = 0.00;
    parameters[1] = 0.00;
    parameters[2] = 0.00;

    dp[0] = 1.00;
    dp[1] = 1.00;
    dp[2] = 1.00;
}

int16_t PIDControl::control(int16_t currentValue)
{
    int16_t c; //correction
    int16_t error = currentValue - targetValue;
    errorSum += error;

    c = (*kp) * error + (*kd) * (error - lastError) + (*ki) * errorSum;

    lastError = error;
    return c;
}
uint16_t PIDControl::PIDcontrol_error(int16_t currentValue)
{
    int16_t c; //correction
    uint16_t error = abs(currentValue - targetValue);
    errorSum += error;

    c = (*kp) * error + (*kd) * (error - lastError) + (*ki) * errorSum;

    lastError = error;
    return error;
}
void PIDControl::clear()
{
    errorSum = 0;
    lastError = 0;
}