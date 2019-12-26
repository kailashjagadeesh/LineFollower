#include "JunctionControl.h"

#include <Arduino.h>

#include "../TesterInterface/TesterInterface.h"
#include "../SensorInterface/SensorInterface.h"
#include "../UltrasonicInterface/UltrasonicInterface.h"
#include "../PushButtonInterface/PushButtonInterface.h"
#include "../MotorDriverInterface/MotorDriverInterface.h"
#include "../LEDInterface/LEDInterface.h"
#include "../configure.h"

#ifndef ALGORITHM_LOGIC
#define ALGORITHM_LOGIC LEFT_LOGIC
#endif

#ifndef MOTOR_OVERSHOOT_SPEED
#define MOTOR_OVERSHOOT_SPEED 100
#endif

#ifndef MOTOR_TURN_SPEED
#define MOTOR_TURN_SPEED 0
#endif

#ifndef MOTOR_EXCESS_TURN_SPEED
#define MOTOR_EXCESS_TURN_SPEED 150
#endif

#ifndef MOTOR_JUNCTION_DELAY
#define MOTOR_JUNCTION_DELAY 500
#endif


JunctionControl::JunctionControl(Sensors &_sensors, Ultrasonic &_ultrasonic, Motor &_motors, MotorPIDControl& _pid) : sensors(_sensors), ultrasonic(_ultrasonic), motors(_motors), pid(_pid)
{
    backSensorState = 0;
    CFState = false;
    blockDetectFlag = false;

    algorithm = ALGORITHM_LOGIC;
    mode = DRY_RUN;
    paused = 0;
}

void JunctionControl::pause(uint32_t m) {
    paused = m + millis();
}

void JunctionControl::updateState()
{
    sensors.readSensors();
    sensors.convertAnalogToDigital();
    uint16_t sensorValues = sensors.digitalValues;

    Debug::print("\nSensor: ");
    Debug::print((uint32_t)(sensorValues), 2);
    Debug::print("\tJunction: ");

    backSensorState = sensorValues & (0b1111);
    backSensorState |= (sensorValues & (0b111100000)) >> 1;
    backSensorStateInverted = (backSensorState * 0x0202020202ULL & 0x010884422010ULL) % 1023; // Contains backSensorState's mirror inversion
    CFState = sensorValues & 0b000010000;

    blockDetectFlag = false; //ultrasonic.detectBlock();
}

void JunctionControl::detect()
{
    updateState();
                      
    if (millis() > paused)
    if (blockDetectFlag == 1)
    {
        control(BL);
        blockDetectFlag = 0;
    }
    else if ((CFState == 0) && (backSensorState & 0b11) == 0b11 && (backSensorStateInverted & 0b11) == 0b11)
    {
        control(T);
    }
    // // Straight Right detection
    else if ((CFState == 1) && (((backSensorState & 0b11100011) == 0b11) ||
                (backSensorState == 0b0110011)))
    {
        control(RS);
    }
    // // Straight Left detection
    else if ((CFState == 1) && (((backSensorStateInverted & 0b11100011) == 0b11) || 
                (backSensorStateInverted == 0b0110011)))
    {
        control(LS);
    }
    else if ((CFState == 0) && ((backSensorState & 0b11000101) == 0b11000100 )) {
        control(T);
    }
    else if ((CFState == 0) && ((backSensorStateInverted & 0b11000101) == 0b11000100 )) {
        control(T);
    }
    // //90 degree right
    else if ((CFState == 0) && (backSensorState <= 0b111) && backSensorState > 0)
    {
        control(R);
    }
    // //90 degrees left
    else if ((CFState == 0) && (backSensorStateInverted <= 0b111) && backSensorStateInverted > 0) 
    {
        control(L);
    }
    else if (CFState == 0 && backSensorState == 0) {
        control(END);
    }
    else if((CFState==1) && (backSensorStateInverted == 255)) {
        control(X);
    }
}

void JunctionControl::control(Junction j)
{
    Debug::print(junctionNames[j]);
//*
    if (mode == DRY_RUN)
    {
        if (algorithm == LEFT_LOGIC)
        {
            switch (j)
            {
            case L:
            case LS:
            case T:
                turnLeft();
                break;

            case R:
                turnRight();
                break;

            case X:
                if (!turnLeft()) {
                    motors.stopMotors();
                    Debug::println("------------------------");
                    while (!PushButtonInterface::read(0)) {
                        LED::toggle(1);
                        delay(1000);
                    }

                    //TODO finish and algorithm
                    //path reduce
                    // change mode
                }
                break;

            case RS:
                sensors.addRightOvershoot();
                pause();
                break;

            case END:
                motors.setLeftSpeed(MOTOR_OVERSHOOT_SPEED);
                motors.setRightSpeed(MOTOR_OVERSHOOT_SPEED);

                motors.setLeftDirection(Motor::FRONT);
                motors.setRightDirection(Motor::FRONT);
                
                while (sensors.rearCenterStatus());

                motors.setLeftSpeed(MOTOR_EXCESS_TURN_SPEED/1.3);
                motors.setRightSpeed(MOTOR_EXCESS_TURN_SPEED/1.3);

                motors.setLeftDirection(Motor::BACK);
                motors.setRightDirection(Motor::FRONT);

                delay(750);

                do  {
                    updateState();
                    Debug::print("left:");
                } while (!(CFState == 1 && (backSensorState & 0b00010000) == 0b00010000));

                pid.setBaseSpeed(180);

                break;
            }
        }
    }
    /**/
}

bool JunctionControl::turnLeft() {
    //overshoot
    sensors.addLeftOvershoot();

    motors.setLeftSpeed(MOTOR_OVERSHOOT_SPEED);
    motors.setRightSpeed(MOTOR_OVERSHOOT_SPEED);

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);
    
    sensors.waitForOvershoot();
    //finish
    updateState();
    if (sensors.digitalValues == 0b111111111) return false;

    //turn
    motors.setLeftDirection(Motor::BACK);
    motors.setRightDirection(Motor::FRONT);
    motors.setLeftSpeed(MOTOR_EXCESS_TURN_SPEED);
    motors.setRightSpeed(MOTOR_TURN_SPEED);

    do
    {
        updateState();
        Debug::print("left");
    } while ((CFState == 1));

    do
    {
        updateState();
        Debug::print("left:");
    } while (!(CFState == 1 && (backSensorState & 0b00010000) == 0b00010000));

    sensors.addLeftOvershoot();
    return true;
}

bool JunctionControl::turnRight() {
    //overshoot
    sensors.addRightOvershoot();

    motors.setLeftSpeed(MOTOR_OVERSHOOT_SPEED);
    motors.setRightSpeed(MOTOR_OVERSHOOT_SPEED);

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);
    
    sensors.waitForOvershoot();
    //finish
    updateState();
    if (sensors.digitalValues == 0b111111111) return false;

    //turning
    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::BACK);
    motors.setRightSpeed(MOTOR_EXCESS_TURN_SPEED);
    motors.setLeftSpeed(MOTOR_TURN_SPEED);

    do
    {
        updateState();  
        Debug::print("right");
    } while (!((backSensorStateInverted & 0b00010000) == 0b00010000) );
    
    sensors.addRightOvershoot();
    return true;
}