#ifndef PID_CONTROL_H
#define PID_CONTROL_H

class PIDControl {
    float kp, kd, ki;
    uint16_t& currentValue;
    uint16_t targetValue;
    int16_t errorSum, lastError;

    public:
    PIDControl(uint16_t& _currentValue, uint16_t _targetValue, float _kp, float _kd, float _ki);
    int16_t control();
    void clear();
    
};

#endif