////////TEST CODE////////
#include "MotorDriverInterface.h"

Motor motor;

#define TESTDELAY 3000

void setup() {
    Serial.begin(9600);
    Serial.println ("Motor driver testing routine:");
    motor.stopMotors();

    Serial.println("Motor: left motor forward full speed");
    motor.setLeftDirection(Motor::FRONT);
    motor.setLeftSpeed(255);
    delay(TESTDELAY);

    Serial.println("Motor: left motor forward half speed");
    motor.setLeftDirection(Motor::FRONT);
    motor.setLeftSpeed(128);
    delay(TESTDELAY);

    Serial.println("Motor: left motor stop");
    motor.stopMotors();
    delay(1000);

    Serial.println("Motor: right motor forward full speed");
    motor.setRightDirection(Motor::FRONT);
    motor.setRightSpeed(255);
    delay(TESTDELAY);

    Serial.println("Motor: right motor forward half speed");
    motor.setRightDirection(Motor::FRONT);
    motor.setRightSpeed(128);
    delay(TESTDELAY);

    Serial.println("Motor: right motor stop");
    motor.stopMotors();
    delay(1000);

    Serial.println("Motor: both motor reverse");
    motor.setLeftDirection(Motor::BACK);
    motor.setRightDirection(Motor::BACK);
    motor.setLeftSpeed(255);
    motor.setRightSpeed(255);

    delay(TESTDELAY);
    
    Serial.println("End test");
}

void loop() {

}