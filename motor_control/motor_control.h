#pragma once

class Motor
{
    uint8_t pinLeftFront, pinLeftBack, pinLeftSpeed;
    uint8_t pinRightFront, pinRightBack, pinRightSpeed;
    uint8_t pinStandby;

public:
    enum Direction
    {
        Front,
        Back
    };

    Motor(const uint8_t pinLeft[], const uint8_t pinRight[], const uint8_t pinStby);
    void setLeftDirection(Direction);
    void setRightDirection(Direction);

    void setLeftSpeed(uint8_t);
    void setRightSpeed(uint8_t);
};

Motor::Motor(const uint8_t pinLeft[], const uint8_t pinRight[], const uint8_t pinStby)
{
    pinLeftFront = pinLeft[0];
    pinLeftBack = pinLeft[1];
    pinLeftSpeed = pinLeft[2];

    pinRightFront = pinRight[0];
    pinRightBack = pinRight[1];
    pinRightSpeed = pinRight[2];

    pinStandby = pinStby;

    pinMode(pinLeftFront, OUTPUT);
    pinMode(pinLeftBack, OUTPUT);
    pinMode(pinLeftSpeed, OUTPUT);

    pinMode(pinRightFront, OUTPUT);
    pinMode(pinRightBack, OUTPUT);
    pinMode(pinRightSpeed, OUTPUT);

    pinMode(pinStandby, OUTPUT);
    digitalWrite(pinStandby, HIGH); // Disable standby pin
}

void Motor::setLeftDirection(Direction dir)
{
    if (dir == Front)
    {
        digitalWrite(pinLeftFront, 1);
        digitalWrite(pinLeftBack, 0);
    }
    else
    {
        digitalWrite(pinLeftFront, 0);
        digitalWrite(pinLeftBack, 1);
    }
}

void Motor::setRightDirection(Direction dir)
{
    if (dir == Front)
    {
        digitalWrite(pinRightFront, 1);
        digitalWrite(pinRightBack, 0);
    }
    else
    {
        digitalWrite(pinRightFront, 0);
        digitalWrite(pinRightBack, 1);
    }
}

void Motor::setLeftSpeed(uint8_t speed)
{
    analogWrite(pinLeftSpeed, speed);
}

void Motor::setRightSpeed(uint8_t speed)
{
    analogWrite(pinRightSpeed, speed);
}