#include"ultrasonic.h"

Ultrasonic ultrasonic;

void setup()
{
  ultrasonic.begin();
   Serial.begin(9600); 
}
void loop()
{
    Serial.println (ultrasonic.measureDistance());
    delay(500);
}
