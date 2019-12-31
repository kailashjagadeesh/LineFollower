//dependencies
#include "src/BluetoothInterface/BluetoothInterface.h"
#include "src/SensorInterface/SensorInterface.h"
#include "src/PIDControl/PIDControl.h"
#include "src/TesterInterface/TesterInterface.h"
#include "src/MotorDriverInterface/MotorDriverInterface.h"
#include "src/LEDInterface/LEDInterface.h"
#include "src/JunctionControl/JunctionControl.h"
#include "src/configure.h"
#include "src/AsyncUltrasonic/AsyncUltrasonic.h"
#include "src/Twiddle/Twiddle.h"

//Bluetooth bluetooth;
Sensors sensors;
Motor motors;
MotorPIDControl pid(PID_IDEAL, 255, motors);
JunctionControl junctionControl(sensors, motors, pid);
volatile bool paused;


void setup()
{
    Debug::begin();
    Debug::useBluetooth();
    LED::init();
    paused = false;

    AsyncUltrasonic::init();
    AsyncUltrasonic::minimumDistance = ULTRASONIC_MIN_DISTANCE;

    Debug::println("Meshmerize Finale!");

    Debug::println("Calibration");
    sensors.calibrate();
    motors.stopMotors();
    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);
    
    while (1) {
        if (PushButtonInterface::read(1)) {
            junctionControl.setAlgorithm(LEFT_LOGIC);
            Debug::println("Using left major algorithm");
            break;
        }
        else if (PushButtonInterface::read(0)) {
            junctionControl.setAlgorithm(RIGHT_LOGIC);
            Debug::println("Using right major algorithm");
            break;
        }
        Debug::print(AsyncUltrasonic::distance);
        Debug::print(" | ");
        Debug::print((int)(AsyncUltrasonic::detectBlock()));
        Debug::print(" | ");
        sensors.updateAllSensors();
        sensors.printAllSensors();
        Debug::println("");

        AsyncUltrasonic::update();
    }

    delay(2000);

    Debug::println("Started");
}

void loop()
{
    AsyncUltrasonic::update();

    if (PushButtonInterface::read(0)) {
        //paused 
        LED::write(0, HIGH);
        Debug::println("---------------paused----------------");
        motors.stopMotors();
        delay(1000);
        while (!PushButtonInterface::read(0)) {
            if (PushButtonInterface::read(1)){
                junctionControl.removeJunction();
                LED::toggle(0);
                delay(500);
            }
        }
        delay(500);
        LED::write(0, LOW);
    }

    int16_t correction = pid.control(sensors.readLine());

    sensors.overshootControl();
    junctionControl.detect();

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);

    if (sensors.nSensorsOnLine <= 4)
        pid.setSpeedBasedOnCorrection(correction);
}