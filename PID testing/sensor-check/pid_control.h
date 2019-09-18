#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>

class PIDControl {
    float kp, kd, ki;
    uint16_t targetValue;
    int16_t errorSum, lastError;

    double parameters[3],dp[3];

    public:
    PIDControl(uint16_t _targetValue, float *_kp, float *_kd, float *_ki);
    int16_t control(int16_t currentValue);
    void clear();
    float *tune();
    
};


#endif