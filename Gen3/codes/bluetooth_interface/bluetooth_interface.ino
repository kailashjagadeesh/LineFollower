

#include"softwareSerial.h"

Bluetooth bluetooth;
void setup()
{
  bluetooth.begin(); 
}
void loop()
{
  bluetooth.println(70);
 
}
