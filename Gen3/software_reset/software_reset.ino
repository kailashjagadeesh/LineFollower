#define button_pin 2
void(* reset_board) (void) = 0; //put reset_board function at address 0

void setup()
{
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(button_pin), button_press, RISING);  // increase counter when speed sensor pin goes High

}
volatile long last_time = 0;
void button_press()
{
    if(millis() - last_time >=3000)
    {
        last_time = millis();
        reset_board();
    }
}

void loop()
{
    Serial.println("Press to button to Reset");
}