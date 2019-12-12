

#include"softwareSerial.h"
softSerial bluetooth(19,18,9600);

void setup()
{
  
}
void loop()
{
  bluetooth.println("T");
}
