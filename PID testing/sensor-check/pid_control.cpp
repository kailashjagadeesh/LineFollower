#include "pid_control.h"

PIDControl::PIDControl(uint16_t _targetValue, float _kp, float _kd, float _ki){
    kp = _kp;
    kd = _kd;
    ki = _ki;

    targetValue = _targetValue;

    errorSum = 0;
    lastError = 0;
}

int16_t PIDControl::control(int16_t currentValue) {
    int16_t c; //correction
    int16_t error =  currentValue - targetValue;
    errorSum += error;

    c = kp * error + kd * (error - lastError) + ki * errorSum;

    lastError = error;
    return c;
}

void PIDControl::clear() {
    errorSum = 0;
    lastError = 0;
}