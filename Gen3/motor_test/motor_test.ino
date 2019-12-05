#include <Stepper.h>

#include <TimerOne.h>


volatile unsigned int counter = 0;
volatile unsigned long int last_time=0;
int pwm_pin = 10;
int dpin1 = 9; 
int dpin2 = 8;

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

  pinMode(pwm_pin, OUTPUT);
  pinMode(dpin1, OUTPUT);
  pinMode(dpin2,OUTPUT);
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt(2), docount, RISING);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void loop()
{
  int potvalue = analogRead(A1);  // Potentiometer connected to Pin A1
  int motorspeed = map(potvalue, 0, 1023, 255, 0);
    //Serial.println(counter);

  analogWrite(pwm_pin, motorspeed);  // set speed of motor (0-255)
  digitalWrite(dpin1, 1);  // set rotation of motor to Clockwise
  digitalWrite(dpin2,0);
}
