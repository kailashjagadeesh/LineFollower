#include "AsyncUltrasonic.h"
#include <Arduino.h>
#include "../LEDInterface/LEDInterface.h"

#ifndef ULTRASONIC_THRESHOLD
#define ULTRASONIC_THRESHOLD 5
#endif

int AsyncUltrasonic::trigPin;
int AsyncUltrasonic::echoPin;
volatile bool AsyncUltrasonic::measuring = false;
volatile uint32_t AsyncUltrasonic::time;
volatile int AsyncUltrasonic::distance = 0;
volatile int AsyncUltrasonic::_distance;
volatile int AsyncUltrasonic::_distanceCount;
Timer AsyncUltrasonic::timer;
int AsyncUltrasonic::minimumDistance;
bool AsyncUltrasonic::_calibrated = false;

void AsyncUltrasonic::init(int t, int e) {
    trigPin = t;
    echoPin = e;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);

    attachInterrupt(echoPin, _interruptHandle, CHANGE);
    timer.every(10, trigger);
}

void AsyncUltrasonic::_interruptHandle() {
    measuring = !measuring;
    if (measuring) time = micros();
    else updateDistance(micros() - time);
}

void AsyncUltrasonic::updateDistance(uint32_t t) {
    int d = (t * 0.034) / 2;
    distance = d;
}

void AsyncUltrasonic::trigger() {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

void AsyncUltrasonic::update() {
    timer.update();
}

bool AsyncUltrasonic::detectBlock() {
    return (distance <= minimumDistance) && distance;
}

