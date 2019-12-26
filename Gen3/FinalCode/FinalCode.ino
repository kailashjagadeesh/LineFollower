//dependencies
#include "src/BluetoothInterface/BluetoothInterface.h"
#include "src/SensorInterface/SensorInterface.h"
#include "src/PIDControl/PIDControl.h"
#include "src/TesterInterface/TesterInterface.h"
#include "src/MotorDriverInterface/MotorDriverInterface.h"
#include "src/LEDInterface/LEDInterface.h"
#include "src/JunctionControl/JunctionControl.h"

///configuration
#define BLACKLINE_LOGIC
#define SERIALD bluetooth
#define PID_CONVFACTOR 100
#define NUM_PIDSENSORS 5
#define PID_IDEAL 2000

Bluetooth bluetooth;

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
    if (PushButtonInterface::read(0))
    {
        motors.stopMotors();
        PushButtonInterface::waitForButton(1);
    }

    int16_t correction = pid.control(sensors.readLine());

    sensors.overshootControl();
    junctionControl.detect();

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);

    pid.setSpeedBasedOnCorrection(correction);
}