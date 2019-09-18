#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>
#include <stdlib.h>
class PIDControl
{
    uint16_t targetValue;
    int16_t errorSum, lastError;

public:
    double *kp, *kd, *ki;
    double parameters[3], dp[3];
    PIDControl(uint16_t _targetValue);
    uint16_t PIDcontrol_error(int16_t currentValue);
    int16_t control(int16_t currentValue);
    void clear();
};

#endif