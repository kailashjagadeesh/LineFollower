#ifndef PID_CONTROL
#define PID_CONTROL

class PIDControl {
    float kp, kd, ki;
    uint16_t& currentValue;
    uint16_t& targetValue;
    int16_t errorSum, lastError;

    public:
    void setup(uint16_t& _currentValue, uint16_t& _targetValue, float _kp, float _kd, float _ki);
    int16_t control();
    
};

#endif