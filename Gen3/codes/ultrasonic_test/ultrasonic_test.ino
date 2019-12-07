#include"ultrasonic.h"

Ultrasonic ultrasonic;

void setup()
{
    
}
void loop()
{
    Serial.println (ultrasonic.measureDistance());
    delay(500);
}