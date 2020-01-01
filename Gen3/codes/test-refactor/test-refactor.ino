#include "src/TesterInterface/TesterInterface.h"
#include "src/AsyncUltrasonic/AsyncUltrasonic.h"
#include "src/SensorInterface/SensorInterface.h"
#include "src/MotorDriverInterface/MotorDriverInterface.h"
#include "src/configure.h"
#include "src/PushButtonInterface/PushButtonInterface.h"

Sensors sensors;
Motor motors;

void setup() {
    Debug::begin();
    Debug::useBluetooth();
    /*Sensor test*/
    AsyncUltrasonic::init();
    AsyncUltrasonic::minimumDistance = ULTRASONIC_MIN_DISTANCE;

    sensors.calibrate();
    /**/

   /* MOTOR TEST 
    motors.stopMotors();
    PushButtonInterface::waitForButton(0);
    /**/
    
}

void loop() { 
    /* Sensor test */ 
    Debug::print(AsyncUltrasonic::distance);
    Debug::print(" | ");
    Debug::print((int)(AsyncUltrasonic::detectBlock()));
    Debug::print(" | ");
    sensors.updateAllSensors();
    sensors.printAllSensors();
    Debug::print("|");
    Debug::println(sensors.readLine());
    
    AsyncUltrasonic::update();

    /**/

    /* MOTOR TEST 

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);
    motors.setLeftSpeed(255);
    motors.setRightSpeed(255);

    PushButtonInterface::waitForButton(0);

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::BACK);
    motors.setLeftSpeed(255);
    motors.setRightSpeed(255);

    PushButtonInterface::waitForButton(0);

    motors.setLeftDirection(Motor::BACK);
    motors.setRightDirection(Motor::BACK);
    motors.setLeftSpeed(255);
    motors.setRightSpeed(255);

    PushButtonInterface::waitForButton(0);

    motors.setLeftDirection(Motor::BACK);
    motors.setRightDirection(Motor::FRONT);
    motors.setLeftSpeed(255);
    motors.setRightSpeed(255);

    PushButtonInterface::waitForButton(0);
    /**/
}