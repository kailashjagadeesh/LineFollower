#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "../PIDControl/PIDControl.h"
#include "../SensorInterface/SensorInterface.h"

namespace Twiddle {
    void autoTune(MotorPIDControl& pid, Sensors &sensors);
}

#endif