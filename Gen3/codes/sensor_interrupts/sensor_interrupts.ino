#include "SensorInterface.h"

volatile bool blockDetectFlag = 0;
Sensors sensors;
void left_sensor_isr()
{
  Serial.println("Left sensor interrupt fired");
  delay(2000);
}
void right_sensor_isr()
{
  Serial.println("Right sensor interrupt fired");
  delay(2000);
}
void middle_sensor_isr()
{
  Serial.println("Middle sensor interrupt fired");
  delay(2000);
}

void setup()
{
  Serial.begin(9600);
  sensors.readSensorsDigital(&backSensorState, &CFState);
  sensors.attachAllInterrupts();
}
void loop()
{
}
