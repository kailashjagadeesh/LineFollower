

#include"softwareSerial.h"


void setup()
{
  bluetoothInit();
}
void loop()
{
  bluetoothPrint("Hello World\n");
}
