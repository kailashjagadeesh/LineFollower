//#define SERIALD bluetooth

#include "softwareSerial.h"
Bluetooth bluetooth;

#define SERIALD bluetooth

#include"ultrasonic.h"
#include "TesterInterface.h"
#include "MotorDriverInterface/MotorDriverInterface.h"
#include "PushButtonInterface/PushButtonInterface.h"

Ultrasonic ultrasonic;
Motor motors;

void setup()
{
   bluetooth.begin();
   Serial.begin(9600);
   ultrasonic.begin();
   motors.stopMotors();

   ultrasonic.calibrate();
   motors.setLeftDirection(Motor::FRONT);
   motors.setRightDirection(Motor::BACK);
   motors.setLeftSpeed(100);
   motors.setRightSpeed(100);
}

void loop()
{
    SERIALD.println (ultrasonic.detectBlock(10)?"Yes" : "No");
}
