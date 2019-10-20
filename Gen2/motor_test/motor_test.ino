#include <TimerOne.h>


volatile unsigned int counter = 0;
volatile unsigned long int last_time=0;
int b1a = 6;  // L9110 B-1A
int b1b = 9;  // L9110 B-1B
int b1c = 8;

void docount()  // counts from the speed sensor
{
  if( millis() - last_time >= 20)
  {
    last_time= millis();
    counter++;  // increase +1 the counter value
  }
}


void timerIsr()
{ 
  Timer1.detachInterrupt();  //stop the timer
  
  Serial.print("Motor Speed: ");
  float rotation = ((float)counter / 20);  // divide by number of holes in Disc
  Serial.print(rotation, DEC);
  Serial.println(" Rotation per seconds");
  counter = 0; //  reset counter to zero
  
   //enable the timer
   Timer1.attachInterrupt( timerIsr ); 
}

void setup()
{
  Serial.begin(9600);

  pinMode(b1a, OUTPUT);
  pinMode(b1b, OUTPUT);
  pinMode(b1c,OUTPUT);
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt(2), docount, RISING);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void loop()
{
  int potvalue = analogRead(A1);  // Potentiometer connected to Pin A1
  int motorspeed = map(potvalue, 0, 1023, 255, 0);
    //Serial.println(counter);

  analogWrite(b1a, motorspeed);  // set speed of motor (0-255)
  digitalWrite(b1b, 1);  // set rotation of motor to Clockwise
  digitalWrite(b1c,0);
}
