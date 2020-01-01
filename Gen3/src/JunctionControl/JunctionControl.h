#ifndef JUNCTION_CONTROL_H
#define JUNCTION_CONTROL_H

#include "../SensorInterface/SensorInterface.h"
#include "../PushButtonInterface/PushButtonInterface.h"
#include "../MotorDriverInterface/MotorDriverInterface.h"
#include "../PIDControl/PIDControl.h"
#include "../Solver/Solver.h"

#include <stdint.h>

///enums used

//sensor nomenclature
enum class sensor
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
};

//mode of run
enum Mode
{
    DRY_RUN,
    ACTUAL_RUN
};

//Junction names
const char junctionNames[][8] = {"R", "L", "X", "T", "LS", "RS", "Y", "BLOCK", "END", "FOLLOW", "FINISH", "INVALID"};

class JunctionControl
{
    uint8_t backSensorState; // Contains state of all sensors except the front extreme one at any time
    bool CFState;            // 1-Means a white line under sensor
    bool blockDetectFlag;
    uint8_t backSensorStateInverted;

    Sensors &sensors;
    Motor &motors;
    MotorPIDControl &pid;

    Algorithm algorithm;
    Mode mode;

    bool turnLeft();
    bool turnRight();
    void turnAroundForBlock();
    void turnAroundForEnd();

    uint32_t paused;
    char path[100];
    int currentJunction;
    uint32_t lastJunctionTime;
    Junction prevJunction;

public:
    void updateState();
    void detect();
    void control(Junction);
    JunctionControl(Sensors &_sensors, Motor &_motors, MotorPIDControl&);
    void pause(uint32_t m = 100);
    void setAlgorithm(Algorithm);
    void addPathChoice(const char*);
    void removeJunction();
};

#endif