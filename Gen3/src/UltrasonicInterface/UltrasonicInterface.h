#ifndef ULTRASONIC_H
#define ULTRASONIC_H

class Ultrasonic
{
    float time;
    float distance;
    int trigPin, echoPin;
    bool calibrated;
    int calibratedValue;

public:
    void begin(int, int);
    float measureDistance();
    bool detectBlock(int thresholdDistance);
    void calibrate();
};

#endif