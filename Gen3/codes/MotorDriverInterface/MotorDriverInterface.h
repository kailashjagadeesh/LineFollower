#ifndef MOTOR_DRIVER_INTERFACE_H
#define MOTOR_DRIVER_INTERFACE_H

//change A with B to swap left and right motors
#define LEFTMOTOR_A


class Motor {
    //pin definitions
    typedef int pin;

    const pin STBY = 13;
    const pin PWMA = 12;
    const pin AIN2 = 11;
    const pin AIN1 = 10;
    const pin BIN1 = 9;
    const pin BIN2 = 8;
    const pin PWMB = 7;

#ifdef LEFTMOTOR_A
    const pin &leftFront = AIN1;
    const pin &rightFront = BIN1;

    const pin &leftBack = AIN2;
    const pin &rightBack = BIN2;

    const pin &leftSpeed = PWMA;
    const pin &rightSpeed = PWMB;
#else
    const pin &leftFront = BIN1;
    const pin &rightFront = AIN1;

    const pin &leftBack = BIN2;
    const pin &rightBack = AIN2;

    const pin &leftSpeed = PWMB;
    const pin &rightSpeed = PWMA;
#endif

public:
    enum Direction {FRONT, BACK};

    Motor();
    void stopMotors();
    void setLeftSpeed(uint8_t);
    void setRightSpeed(uint8_t);
    void setLeftDirection(Direction);
    void setRightDirection(Direction);
};

Motor::Motor() {
    pinMode(leftFront, OUTPUT);
    pinMode(leftBack, OUTPUT);
    pinMode(leftSpeed, OUTPUT);

    pinMode(rightFront, OUTPUT);
    pinMode(rightBack, OUTPUT);
    pinMode(rightSpeed, OUTPUT);

    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH);
}

//stops the motors
void Motor::stopMotors() {
    digitalWrite(STBY,LOW);

    setLeftSpeed(0);
    setRightSpeed(0);

    digitalWrite(STBY,HIGH);
}

//setLeftSpeed(speed: the speed to be set : 0 <= speed <= 255)
void Motor::setLeftSpeed(uint8_t speed)
{
    analogWrite(leftSpeed, speed);
}

//setRightSpeed(speed: the speed to be set : 0 <= speed <= 255)
void Motor::setRightSpeed(uint8_t speed)
{
    analogWrite(rightSpeed, speed);
}

void Motor::setLeftDirection(Direction dir)
{
    if (dir == FRONT)
    {
        digitalWrite(leftFront, 1);
        digitalWrite(leftBack, 0);
    }
    else
    {
        digitalWrite(leftFront, 0);
        digitalWrite(leftBack, 1);
    }
}

void Motor::setRightDirection(Direction dir)
{
    if (dir == FRONT)
    {
        digitalWrite(rightFront, 1);
        digitalWrite(rightBack, 0);
    }
    else
    {
        digitalWrite(rightFront, 0);
        digitalWrite(rightBack, 1);
    }
}
#endif