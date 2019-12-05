#include <TimerOne.h>
#include <Grove_Motor_Driver_TB6612FNG.h>
#include <I2Cdev.h>

#define intr_count90 100;
#define voltage_scale_factor 3.3f/5.0f

volatile unsigned int counter_left = 0;
volatile unsigned int counter_right = 0;

 //Left Motor
int pwm_pin1 =10;
int dpin1 = 9; 
int dpin2 = 8;

//Right Motor
int pwm_pin2=5;
int dpin3= 7;
int dpin4= 6;

void docount_left()  // counts from the speed sensor for left motor
{
      counter_left++;
}
void docount_right() // counts from the speed sensor for right motor
{
      counter_right++;
}

void left_90()
{
 static int temp_count= counter_left;
    // Set left motor forward
  digitalWrite(dpin1, HIGH);
  digitalWrite(dpin2, LOW);
 
  // Set right motor reverse
  digitalWrite(dpin3, LOW);
  digitalWrite(dpin4, HIGH);
  if(temp_count < counter_left + intr_count90)
  {
    analogWrite(pwm_pin1,255* voltage_scale_factor);
    left_90();
  }
}

void right_90()
{
 static int temp_count= counter_right;
    // Set left motor reverse
  digitalWrite(dpin1, LOW);
  digitalWrite(dpin2, HIGH);
 
  // Set right motor forward
  digitalWrite(dpin3, HIGH);
  digitalWrite(dpin4, LOW);
  if(temp_count < counter_left + intr_count90)
  {
    analogWrite(pwm_pin2,255 * voltage_scale_factor);
    right_90();
  }
}

void forward()
{

  digitalWrite(dpin1, HIGH);
  digitalWrite(dpin2, LOW);
 
  digitalWrite(dpin3, HIGH);
  digitalWrite(dpin4, LOW);

void timerIsr()
{ 
  Timer1.detachInterrupt();  //stop the timer
  
  Serial.print("Left motor Interrupt count is:  ");
  Serial.println(left_counter, DEC); 

  
  Serial.print("Right motor Interrupt count is:  ");
  Serial.println(right_counter, DEC); 
   //enable the timer
   Timer1.attachInterrupt( timerIsr ); 
}

void setup()
{
  Serial.begin(9600);

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
 left_90();
 delay(1000);
 right_90();
 delay(1000);

}