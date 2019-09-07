#ifndef PID_CONTROL
#define PID_CONTROL

class PIDControl {
    float kp, kd, ki;

    public:
    PIDControl (float _kp, float _kd, float _ki);
    
};

#endif