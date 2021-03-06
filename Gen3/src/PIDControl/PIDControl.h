#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include<stdint.h>

#include "../MotorDriverInterface/MotorDriverInterface.h"

class PIDControl
{
    protected:
    float targetValue;
    float lastError;
    double errorSum;
    uint32_t increaseTime;
    bool tuning;

public:
    double *kp, *kd, *ki;
    double parameters[6], dp[3];
    PIDControl(uint16_t _targetValue);
    float getError(int16_t currentValue);
    int16_t control(int16_t currentValue);
    void clear();
    void increasePID(uint32_t m = 200);
};

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
    void setBaseSpeed(uint8_t speed);
};

void autoTunePID(MotorPIDControl& pid);

#endif