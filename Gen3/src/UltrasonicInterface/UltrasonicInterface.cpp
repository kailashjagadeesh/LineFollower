#include "UltrasonicInterface.h"

#include<Arduino.h>

#include "../TesterInterface/TesterInterface.h"
#include "../PushButtonInterface/PushButtonInterface.h"

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
    Debug::println("Keep the block at appropriate dist.");
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