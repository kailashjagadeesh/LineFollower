void(* reset_board) (void) = 0; //put reset_board function at address 0

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    Serial.println("Resetting");
    delay(500);
    reset_board();

    Serial.println("This line not printed");
}