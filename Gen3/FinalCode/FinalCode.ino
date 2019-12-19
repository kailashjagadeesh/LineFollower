///configuration
#define BLACKLINE_LOGIC
#define SERIALD bluetooth
#define PID_CONVFACTOR 100
#define NUM_PIDSENSORS 9
#define PID_IDEAL 4000
#define PUSH_BUTTON 50

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

void setup() {
    bluetooth.begin();
    LED::init();
    pinMode(PUSH_BUTTON, INPUT_PULLUP);
    SERIALD.println("Meshmerize Finale!");

    SERIALD.println ("Calibration");
    sensors.calibrate();

    SERIALD.println("Press the button to begin!");
    while (digitalRead(PUSH_BUTTON));

    SERIALD.println("Started");
    motors.stopMotors();
}

void loop() {
    int16_t correction = pid.control(sensors.readLine());

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);

    junctionControl.detect();

    pid.setSpeedBasedOnCorrection(correction);
}