#ifndef JUNCTION_CONTROL_H
#define JUNCTION_CONTROL_H

#ifndef ALGORITHM_LOGIC
#define ALGORITHM_LOGIC LEFT_LOGIC
#endif

#ifndef MOTOR_TURN_SPEED
#define MOTOR_TURN_SPEED 100
#endif

#include "SensorInterface.h"
#include "ultrasonic.h"

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
} mode;

//Junction names
const char junctionNames[][8] = {"R", "L", "X", "T", "LS", "RS", "Y", "BLOCK", "END", "FOLLOW", "FINISH", "INVALID"};

class JunctionControl
{
    uint8_t backSensorState; // Contains state of all sensors except the front extreme one at any time
    bool CFState;            // 1-Means a white line under sensor
    bool blockDetectFlag;

    Sensors &sensors;
    Ultrasonic &ultrasonic;
    Motor &motors;

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

    backSensorState = sensorValues & (0b1111);
    backSensorState |= (sensorValues & (0b111100000)) >> 1;

    SERIALD.print("back sensors: ");
    SERIALD.println((uint32_t)(backSensorState), BIN);

    CFState = sensorValues & 0b000010000;
    SERIALD.print("CF state: ");
    if (CFState)
        SERIALD.println("1");
    else
        SERIALD.println("0");

    blockDetectFlag = false; //ultrasonic.detectBlock(); TODO
}

void JunctionControl::detect()
{
    updateState();
    uint8_t backSensorStateInverted = (backSensorState * 0x0202020202ULL & 0x010884422010ULL) % 1023; // Contains backSensorState's mirror inversion
                                                                                                      // Block detected condition
    if (blockDetectFlag == 1)
    {
        control(BL);
        blockDetectFlag = 0;
    }
    // Normal Line follow
    else if (((CFState == 1) && (backSensorState == 0b00011000)) || ((CFState == 0) && (backSensorState == 0b00110000) || (backSensorState == 0b00001100)))
    {
        return; // Return control back to PID
    }
    else if ((CFState == 0) && ((backSensorState == 0b00111100) || (backSensorState == 0b00111110) || (backSensorState == 0b01111100)))
    {
        control(Y);
    }
    else if ((CFState == 1) && (backSensorState == 255))
    {
        control(X);
    }
    else if ((CFState == 0) && (backSensorState == 255))
    {
        control(T);
    }
    // Straight Right detection
    else if ((CFState == 1) && ((backSensorState == 0b00001111) || (backSensorState == 0b00011111)))
    {
        control(RS);
    }
    // Straight Left detection
    else if ((CFState == 1) && ((backSensorStateInverted == 0b00001111) || (backSensorStateInverted == 0b00011111)))
    {
        control(LS);
    }
    //90 degree right
    else if ((CFState == 0) && ((backSensorState == 0b00001111) || (backSensorState == 0b00011111)))
    {
        control(R);
    }
    // Straight Left detection
    else if ((CFState == 0) && ((backSensorStateInverted == 0b00001111) || (backSensorStateInverted == 0b00011111)))
    {
        control(L);
    }
}

void JunctionControl::control(Junction j)
{
    SERIALD.print("Junction: ");
    SERIALD.println(junctionNames[j]);
    if (mode == DRY_RUN)
    {
        if (algorithm == LEFT_LOGIC)
        {
            switch (j)
            {
            case L:
                do
                {
                    motors.setLeftDirection(Motor::BACK);
                    motors.setRightDirection(Motor::FRONT);
                    motors.setLeftSpeed(MOTOR_TURN_SPEED);
                    motors.setRightSpeed(MOTOR_TURN_SPEED);

                    updateState();

                } while (!((backSensorState == 0b00011000) && (CFState == 1)));
                break;
            }
        }
    }
}
#endif