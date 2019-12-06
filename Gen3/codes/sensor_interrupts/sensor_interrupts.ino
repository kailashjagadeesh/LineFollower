

// #define FOLLOW_WHITE 

int left_end_sensor=18;
int right_end_sensor=19;
int middle_sensor =20;

void setup() {
  Serial.begin(9600);
}

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

void loop() {
  #ifdef FOLLOW_WHITE
  
      {
        attachInterrupt(digitalPinToInterrupt(left_end_sensor),left_sensor_isr,HIGH);
      attachInterrupt(digitalPinToInterrupt(right_end_sensor),right_sensor_isr,HIGH);
      attachInterrupt(digitalPinToInterrupt(middle_sensor),middle_sensor_isr,HIGH);}
  #else
      {attachInterrupt(digitalPinToInterrupt(left_end_sensor),left_sensor_isr,LOW);
      attachInterrupt(digitalPinToInterrupt(right_end_sensor),right_sensor_isr,LOW);
      attachInterrupt(digitalPinToInterrupt(middle_sensor),middle_sensor_isr,LOW);}
   #endif
   delay(2000);
}
