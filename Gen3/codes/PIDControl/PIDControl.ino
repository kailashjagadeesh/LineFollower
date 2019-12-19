#define BLACKLINE_LOGIC

#define PID_CONVFACTOR 100
#define NUM_PIDSENSORS 9
#define PID_IDEAL 4000

#include "PIDControl.h"
#include "SensorInterface.h"
#include "MotorDriverInterface.h"

Sensors sensors;
Motor motors;
MotorPIDControl pid(PID_IDEAL, 255, motors);

void setup() {
    Serial.begin(9600);
    LED::init();
    motors.stopMotors();

    sensors.calibrate();

    SERIALD.println("Keep on the starting position");
    delay(5000);
}

void loop () {
    int16_t correction = pid.control(sensors.readLine());
    // SERIALD.print("pidCorrection: ");
    // SERIALD.println(correction);

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);

    pid.setSpeedBasedOnCorrection(correction);
}