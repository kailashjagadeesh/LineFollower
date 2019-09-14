#include "pid_control.h"

void PIDControl::setup(uint16_t& _currentValue, uint16_t& _targetValue, float _kp, float _kd, float _ki):currentValue(_currentValue), targetValue(_targetValue){
    kp = _kp;
    kd = _kd;
    ki = _ki;

    errorSum = 0;
    lastError = 0;
}

int16_t PIDControl::control() {
    int16_t c; //correction
    int16_t error = currentValue - targetValue;
    errorSum += error;

    c = kp * error + kd * (error - lastError) + ki * errorSum;

    lastError = error;
    return c;
}