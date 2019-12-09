volatile unsigned int left_counter = 0;
volatile unsigned int right_counter = 0;
volatile unsigned long int last_time=0;
int pwm_pin1 =12;
int dpin1 = 10; 
int dpin2 = 11;

//Right Motor
int pwm_pin2=7;
int dpin4= 9;
int dpin3= 8;

int STBY=13; //Standby pin on the motor driver

void docount_left()  // counts from the speed sensor
{
  if( millis() - last_time >= 20)
  {
    last_time= millis();
    left_counter++;  // increase +1 the counter value
  }
}
void docount_right()  // counts from the speed sensor
{
  if( millis() - last_time >= 20)
  {
    last_time= millis();
    right_counter++;  // increase +1 the counter value
  }
}



/* void timerIsr()
{ 
  ////timer1.detachInterrupt();  //stop the timer
  
  Serial.print("Motor Speed: ");
  float rotation = ((float)counter / 20);  // divide by number of holes in Disc
  Serial.print(rotation, DEC);
  Serial.println(" Rotation per seconds");
  counter = 0; //  reset counter to zero
  
   //enable the timer
   //timer1.attachInterrupt( timerIsr ); 
} */

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
  //timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt(31), docount_left, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(33), docount_right, RISING);  // increase counter when speed sensor pin goes High
  digitalWrite(STBY, HIGH);
  analogWrite(pwm_pin1, 255);  // set speed of motor (0-255)
  digitalWrite(dpin1, 1);  // set rotation of motor to Clockwise
  digitalWrite(dpin2,0);
  
  analogWrite(pwm_pin2, 255);  // set speed of motor (0-255)
  digitalWrite(dpin4, 1);  // set rotation of motor to Clockwise
  digitalWrite(dpin3,0);
  //timer1.attachInterrupt( timerIsr ); // enable the timer
}

void loop()
{ delay(5000);
  float rotation_left = ((float)left_counter / 20);  // divide by number of holes in Disc
  float rotation_right=  ((float)right_counter / 20);
  
  Serial.print(rotation_left * 12, DEC);
  Serial.println(" Rotation per minute (left motor)");
  Serial.print(rotation_right * 12, DEC);
  Serial.println(" Rotation per minute (right motor)");
  left_counter = 0; //  reset counter to zero
  right_counter=0;
    //Serial.println(counter);

  
}
