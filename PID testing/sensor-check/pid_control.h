#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>
#include <stdlib.h>
class PIDControl
{
    float targetValue;
    float errorSum, lastError;

public:
    double *kp, *kd, *ki;
    double parameters[3], dp[3];
    PIDControl(uint16_t _targetValue);
    float getError(int16_t currentValue);
    int16_t control(int16_t currentValue);
    void clear();
};

#endif