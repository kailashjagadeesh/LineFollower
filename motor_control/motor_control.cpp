#include "motor_control.h"

Motor::Motor (const uint8_t pinLeft[], const uint8_t pinRight[]) {
    pinLeftFront = pinLeft[0];
    pinLeftBack = pinLeft[1];
    pinLeftSpeed = pinLeft[2];

    pinRightFront = pinRight[0];
    pinRightBack = pinRight[1];
    pinRightSpeed = pinRight[2];

    pinMode(pinLeftFront, OUTPUT);
    pinMode(pinLeftBack, OUTPUT);
    pinMode(pinLeftSpeed, OUTPUT);

    pinMode(pinRightFront, OUTPUT);
    pinMode(pinRightBack, OUTPUT);
    pinMode(pinRightSpeed, OUTPUT);
}

void Motor::setLeftDirection(Direction dir) {
    if (dir == Front) {
        digitalWrite(pinLeftFront, HIGH);
        digitalWrite(pinLeftBack, LOW);
    }
    else {
        digitalWrite(pinLeftFront, LOW);
        digitalWrite(pinLeftBack, HIGH);
    }
}

void Motor::setRightDirection(Direction dir) {
    if (dir == Front) {
        digitalWrite(pinRightFront, HIGH);
        digitalWrite(pinRightBack, LOW);
    }
    else {
        digitalWrite(pinRightFront, LOW);
        digitalWrite(pinRightBack, HIGH);
    }
}

void Motor::setLeftSpeed (uint8_t speed) {
    analogWrite(pinLeftSpeed, speed);
}

void Motor::setRightSpeed (uint8_t speed) {
    analogWrite(pinRightSpeed, speed);
}