#define SERIALD bluetooth

#include"ultrasonic.h"
#include "softwareSerial.h"
#include "TesterInterface.h"

Ultrasonic ultrasonic;
Bluetooth bluetooth;

void setup()
{
   bluetooth.begin();
}

void loop()
{
    SERIALD.println ((ultrasonic.measureDistance()));
    delay(500);
}
