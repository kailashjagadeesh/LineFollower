#include <TimerOne.h>

#define interrupt_count90 90
#define interrupt_count180 150
#define interrupt_count360 300 

volatile unsigned int counter_left = 0;
volatile unsigned int counter_right = 0;

 //Left Motor
int pwm_pin1 =12;
int dpin1 = 10; 
int dpin2 = 11;

//Right Motor
int pwm_pin2=7;
int dpin3= 9;
int dpin4= 8;

int STBY=13; //Standby pin on the motor driver

void docount_left()  // counts from the speed sensor for left motor
{
      counter_left++;
}
void docount_right() // counts from the speed sensor for right motor
{
      counter_right++;
}

void turn(int angle, char dir)
{
 counter_left=0;
 counter_right=0;

 int interrupt_count;
 if(dir=='r')
 {
 digitalWrite(STBY,HIGH); //disable standby
    // Set left motor forward
  digitalWrite(dpin1, HIGH);
  digitalWrite(dpin2, LOW);
 
  // Set right motor reverse
  digitalWrite(dpin4, LOW);
  digitalWrite(dpin3, HIGH);
 }
 if(dir=='l')
  {
  digitalWrite(STBY,HIGH); //disable standby
    // Set left motor reverse
  digitalWrite(dpin1, LOW);
  digitalWrite(dpin2, HIGH);
 
  // Set right motor forward
  digitalWrite(dpin4, HIGH);
  digitalWrite(dpin3, LOW);
  }
  if(angle== 90)
    interrupt_count= interrupt_count90;
  if(angle==180)
    interrupt_count= interrupt_count180;
  if(angle== 360)
    interrupt_count= interrupt_count360;
    
 while(interrupt_count > counter_left && interrupt_count> counter_right)
  {
  if(counter_right <interrupt_count)
    analogWrite(pwm_pin1,255 );
  else
    stop();
  if(counter_left < interrupt_count)
    analogWrite(pwm_pin2,255 );
  else
    stop();
   } 
 }

void forward()
{
  counter_left=0;
  counter_right=0;
  digitalWrite(STBY,HIGH); //disable standby
  
  analogWrite(pwm_pin1,255);
  digitalWrite(dpin1, HIGH);
  digitalWrite(dpin2, LOW);
 
  analogWrite(pwm_pin2,255);
  digitalWrite(dpin4, HIGH);
  digitalWrite(dpin3, LOW);
}

void stop(){
//enable standby
digitalWrite(STBY, LOW);
}

void timerIsr()
{ 
  Timer1.detachInterrupt();  //stop the timer
  
  Serial.print("Left motor Interrupt count is:");
  Serial.println(counter_left, DEC); 

  
  Serial.print("Right motor Interrupt count is:");
  Serial.println(counter_right, DEC); 
   //enable the timer
   Timer1.attachInterrupt( timerIsr ); 
}

void setup()
{
  Serial.begin(9600);

  pinMode(STBY, OUTPUT);

  pinMode(pwm_pin1, OUTPUT);
  pinMode(dpin1, OUTPUT);
  pinMode(dpin2,OUTPUT);

  
  pinMode(pwm_pin2, OUTPUT);
  pinMode(dpin3, OUTPUT);
  pinMode(dpin4,OUTPUT);
  
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt(2), docount_left, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(3), docount_right, RISING);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer

}

void loop() {

   forward();
 delay(1000);
 stop(); //stop
 delay(250); //hold for 250ms until move again 


 /*turn(90,'r');
 stop(); //stop
 delay(250); //hold for 250ms until move again
 
   /*forward();
 delay(1000);
 stop(); //stop
 delay(250); //hold for 250ms until move again  


 turn(180,'l');
 stop(); //stop
 delay(250); //hold for 250ms until move again 
 
  forward();
 delay(1000);
 stop(); //stop
 delay(250); //hold for 250ms until move again 

  turn(360,'l');
 stop(); //stop
 delay(250); //hold for 250ms until move again */
 }
