#pragma once

class Motor {
    uint8_t pinLeftFront, pinLeftBack, pinLeftSpeed;
    uint8_t pinRightFront, pinRightBack, pinRightSpeed;
    
    public:
    enum Direction {
        Front, Back
    };

    Motor (const uint8_t pinLeft[], const uint8_t pinRight[]);
    void setLeftDirection (Direction);
    void setRightDirection (Direction);

    void setLeftSpeed(uint8_t);
    void setRightSpeed(uint8_t);
};