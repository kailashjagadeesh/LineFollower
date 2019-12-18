#include"ultrasonic.h"

Ultrasonic ultrasonic;

void setup()
{
   Serial.begin(9600); 
}
void loop()
{
    Serial.println (ultrasonic.measureDistance());
    delay(500);
}
