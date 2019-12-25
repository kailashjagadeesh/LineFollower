#ifndef JUNCTION_CONTROL_H
#define JUNCTION_CONTROL_H

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
#define MOTOR_JUNCTION_DELAY 75
#endif

#include "SensorInterface.h"
#include "ultrasonic.h"
#include "PushButtonInterface.h"

///enums used

//sensor nomenclature
enum sensor
{
    RB2 = 0,
    RB1,
    RM,
    RF,
    LF,
    LM,
    LB1,
    LB2
};

//Junction nomenclature
enum Junction
{
    R,
    L,
    X,
    T,
    LS,
    RS,
    Y,
    BL,
    END,
    FOLLOW,
    FINISH,
    INVALID
};

//algorithm variations
enum Algorithm
{
    LEFT_LOGIC,
    RIGHT_LOGIC
} algorithm = ALGORITHM_LOGIC;

//mode of run
enum Mode
{
    DRY_RUN,
    ACTUAL_RUN
} mode = DRY_RUN;

//Junction names
const char junctionNames[][8] = {"R", "L", "X", "T", "LS", "RS", "Y", "BLOCK", "END", "FOLLOW", "FINISH", "INVALID"};

class JunctionControl
{
    uint8_t backSensorState; // Contains state of all sensors except the front extreme one at any time
    bool CFState;            // 1-Means a white line under sensor
    bool blockDetectFlag;
    uint8_t backSensorStateInverted;

    Sensors &sensors;
    Ultrasonic &ultrasonic;
    Motor &motors;

    void turnLeft();
    void turnRight();

public:
    void updateState();
    void detect();
    void control(Junction);
    JunctionControl(Sensors &_sensors, Ultrasonic &_ultrasonic, Motor &_motors);
};

JunctionControl::JunctionControl(Sensors &_sensors, Ultrasonic &_ultrasonic, Motor &_motors) : sensors(_sensors), ultrasonic(_ultrasonic), motors(_motors)
{
    backSensorState = 0;
    CFState = false;
    blockDetectFlag = false;
}

void JunctionControl::updateState()
{
    sensors.readSensors();
    sensors.convertAnalogToDigital();
    uint16_t sensorValues = sensors.digitalValues;

    SERIALD.print("\nSensor: ");
    SERIALD.print((uint32_t)(sensorValues), 2);
    SERIALD.print("\tJunction: ");

    backSensorState = sensorValues & (0b1111);
    backSensorState |= (sensorValues & (0b111100000)) >> 1;
    backSensorStateInverted = (backSensorState * 0x0202020202ULL & 0x010884422010ULL) % 1023; // Contains backSensorState's mirror inversion
    CFState = sensorValues & 0b000010000;

    blockDetectFlag = false; //ultrasonic.detectBlock();
}

void JunctionControl::detect()
{
    updateState();
                                                                                                      // Block detected condition
    if (blockDetectFlag == 1)
    {
        control(BL);
        blockDetectFlag = 0;
    }
    else if (sensors.nSensorsOnLine > 3)
    {
        if (0)
            ;
        // else if ((CFState == 1) && (backSensorState & 0b11000011) == 0b11000011)
        // {
        //     control(X);
        // }
        else if ((CFState == 0) && (backSensorState & 0b11) > 0 && (backSensorStateInverted & 0b11) > 0)
        {
            control(T);
        }
        // // Straight Right detection
        else if (((CFState == 1) && (backSensorState & 0b11100111) == 0b111) ||
                    (backSensorState == 0b0110011))
        {
            control(RS);
        }
        // // Straight Left detection
        else if (((CFState == 1) && (backSensorStateInverted & 0b11100111) == 0b111) || 
                    (backSensorStateInverted == 0b0110011))
        {
            control(LS);
        }
        // //90 degree right
        else if ((CFState == 0) && (backSensorState & 0b11100111) == 0b111)
        {
            control(R);
        }
        // //90 degrees left
        else if ((CFState == 0) && (backSensorStateInverted & 0b11100111) == 0b111)
        {
            control(L);
        }
    } 
    else if (sensors.nSensorsOnLine == 0) {
        control(END);
    }
}

void JunctionControl::control(Junction j)
{
    SERIALD.print(junctionNames[j]);
//*
    if (mode == DRY_RUN)
    {
        if (algorithm == LEFT_LOGIC)
        {
            switch (j)
            {
            case L:
            case T:
                turnLeft();
                break;

            case R:
                turnRight();
                break;

            case X:
            case LS:
                motors.setLeftSpeed(MOTOR_OVERSHOOT_SPEED);
                motors.setRightSpeed(MOTOR_OVERSHOOT_SPEED);

                motors.setLeftDirection(Motor::FRONT);
                motors.setRightDirection(Motor::FRONT);

                delay(MOTOR_JUNCTION_DELAY);

                motors.setLeftDirection(Motor::BACK);
                motors.setRightDirection(Motor::FRONT);
                motors.setLeftSpeed(MOTOR_EXCESS_TURN_SPEED);
                motors.setRightSpeed(MOTOR_TURN_SPEED);

                do
                {
                    updateState();
                    SERIALD.print("left");
                } while ((CFState == 1));

                do
                {
                    updateState();
                    SERIALD.print("left");
                } while (!(CFState == 1 && (backSensorState & 0b00010000 == 0b00010000)));

                motors.stopMotors();
                PushButtonInterface::waitForButton(0);
                break;

            case RS:
                break;

            case END:
                motors.setLeftSpeed(MOTOR_OVERSHOOT_SPEED);
                motors.setRightSpeed(MOTOR_OVERSHOOT_SPEED);

                motors.setLeftDirection(Motor::FRONT);
                motors.setRightDirection(Motor::FRONT);

                delay(MOTOR_JUNCTION_DELAY);

                motors.setLeftSpeed(MOTOR_EXCESS_TURN_SPEED);
                motors.setRightSpeed(MOTOR_EXCESS_TURN_SPEED);

                motors.setLeftDirection(Motor::FRONT);
                motors.setRightDirection(Motor::BACK);

                do {
                    updateState();
                    SERIALD.print("left");
                } while ((CFState == 0));

                break;
            }
        }
    }
    /**/
}

void JunctionControl::turnLeft() {
    //overshoot
    sensors.increaseJunction(sensors.overshootLeftPin);

    motors.setLeftSpeed(MOTOR_OVERSHOOT_SPEED);
    motors.setRightSpeed(MOTOR_OVERSHOOT_SPEED);

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);
    
    LED::write(0, HIGH);
    while (!sensors.overshootCompleted());
    motors.stopMotors();
    LED::write(0, LOW);

    // while (!sensors.readOvershootSensors()) {
    //     SERIALD.print(sensors.rawOvershootSensorValue(0));
    //     SERIALD.println(sensors.rawOvershootSensorValue(1));
    // }
    // SERIALD.println("-");
    // SERIALD.println(sensors.readOvershootSensors());
    // SERIALD.println("-");
    // while (sensors.readOvershootSensors()) {
    //     SERIALD.print(sensors.rawOvershootSensorValue(0));
    //     SERIALD.println(sensors.rawOvershootSensorValue(1));
    // }

    //turn
    motors.setLeftDirection(Motor::BACK);
    motors.setRightDirection(Motor::FRONT);
    motors.setLeftSpeed(MOTOR_EXCESS_TURN_SPEED);
    motors.setRightSpeed(MOTOR_TURN_SPEED);

    do
    {
        updateState();
        SERIALD.print("left:");
    } while (!(CFState == 1 && (backSensorState & 0b00010000) == 0b00010000));

    sensors.increaseJunction(sensors.overshootLeftPin);
}

void JunctionControl::turnRight() {
    //overshoot
    sensors.increaseJunction(sensors.overshootRightPin);

    motors.setLeftSpeed(MOTOR_OVERSHOOT_SPEED);
    motors.setRightSpeed(MOTOR_OVERSHOOT_SPEED);

    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::FRONT);
    
    LED::write(0, HIGH);
    while (!sensors.overshootCompleted());
    motors.stopMotors();
    LED::write(0, LOW);
    
    // while (!sensors.readOvershootSensors()) {
    //     SERIALD.print(sensors.rawOvershootSensorValue(0));
    //     SERIALD.println(sensors.rawOvershootSensorValue(1));
    // }
    // SERIALD.println("-");
    // SERIALD.println(sensors.readOvershootSensors());
    // SERIALD.println("-");
    // while (sensors.readOvershootSensors()) {
    //     SERIALD.print(sensors.rawOvershootSensorValue(0));
    //     SERIALD.println(sensors.rawOvershootSensorValue(1));
    // }

    //turning
    motors.setLeftDirection(Motor::FRONT);
    motors.setRightDirection(Motor::BACK);
    motors.setRightSpeed(MOTOR_EXCESS_TURN_SPEED);
    motors.setLeftSpeed(MOTOR_TURN_SPEED);

    do
    {
        updateState();
        SERIALD.print("right");
    } while (!(CFState == 1 && (backSensorStateInverted & 0b00010000) == 0b00010000 ));
    
    sensors.increaseJunction(sensors.overshootRightPin);
}
#endif