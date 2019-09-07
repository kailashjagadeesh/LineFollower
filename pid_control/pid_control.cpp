#include "pid_control.h"

PIDControl::PIDControl(float _kp, float _kd, float _ki) {
    kp = _kp;
    kd = _kd;
    ki = _ki;
}