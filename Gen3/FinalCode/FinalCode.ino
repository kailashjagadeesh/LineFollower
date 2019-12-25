///configuration
#define BLACKLINE_LOGIC
#define SERIALD bluetooth
#define PID_CONVFACTOR 100
#define NUM_PIDSENSORS 9
#define PID_IDEAL 4000

//dependencies
#include "softwareSerial.h"

Bluetooth bluetooth;

#include "SensorInterface.h"
#include "PIDControl.h"
#include "TesterInterface.h"
#include "MotorDriverInterface.h"
#include "LEDInterface.h"
#include "JunctionControl.h"

//Bluetooth bluetooth;
Sensors sensors;
Motor motors;
Ultrasonic ultrasonic;
MotorPIDControl pid(PID_IDEAL, 255, motors);
JunctionControl junctionControl(sensors, ultrasonic, motors);

void setup()
{
    bluetooth.begin();
    LED::init();
    SERIALD.println("Meshmerize Finale!");

    SERIALD.println("Calibration");
    sensors.calibrate();

    SERIALD.println("Press the button to begin!");
    PushButtonInterface::waitForButton(0);
    delay(2000);

    SERIALD.println("Started");
    motors.stopMotors();
}

void loop()
{
    if (PushButtonInterface::readState(0))
    {
        motors.stopMotors();
        PushButtonInterface::waitForButton(1);
    }

    int16_t correction = pid.control(sensors.readLine());

    junctionControl.detect();

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);

    pid.setSpeedBasedOnCorrection(correction);
}