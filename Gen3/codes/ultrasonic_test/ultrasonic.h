#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include "PushButtonInterface/PushButtonInterface.h"

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
// Initializes GPIOs for using ultrasonic
void Ultrasonic ::begin(int trig = 24, int echo = 22)
{
    trigPin = trig;
    echoPin = echo;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
}

void Ultrasonic::calibrate() {
    SERIALD.println("Keep the block at appropriate dist.");
    PushButtonInterface::waitForButton(0);

    float dist = 0;
    int i;
    for (i = 0; i < 10; i++)
        dist += measureDistance();

    calibratedValue = dist / i;
    calibrated = true;
}

// Returns distance measured by the sensor in CMs
float Ultrasonic ::measureDistance()
{
    digitalWrite(trigPin, HIGH);

    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);

    time = (float)pulseIn(echoPin, HIGH);

    distance = (time * 0.034) / 2;
    return distance;
}
//Argument holds threshold distance for deciding if block is present
bool Ultrasonic :: detectBlock(int thresholdDistance = 5)
{
    if (calibrated)
        thresholdDistance = calibratedValue;

    return  (measureDistance() <= thresholdDistance) 
        && (measureDistance() <= thresholdDistance) 
        && (measureDistance() <= thresholdDistance)
        && (measureDistance() <= thresholdDistance)
        && (measureDistance() <= thresholdDistance)
    ;
}
#endif