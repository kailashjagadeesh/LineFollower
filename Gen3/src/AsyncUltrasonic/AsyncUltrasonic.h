#ifndef ASYNC_ULTRASONIC_H
#define ASYNC_ULTRASONIC_H
#include <stdint.h>
#include "../Timer/Timer.h"

namespace AsyncUltrasonic {
    extern int trigPin;
    extern int echoPin;

    extern volatile bool measuring;
    extern volatile uint32_t time;
    
    extern volatile int distance;
    extern volatile int _distance;
    extern volatile int _distanceCount;

    extern int calibratedDistance;
    extern bool _calibrated;

    extern Timer timer;

    void init(int t = 24, int e = 22);

    void _interruptHandle();
    void trigger();

    void updateDistance(uint32_t);
    void update();

    void calibrate();

    bool detectBlock();
}
#endif