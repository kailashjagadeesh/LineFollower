const int digitalPins[] = {6, 5, 4, 3, 40, 28, 30, 26, 32, 38, 34, 36};
const int analogPins[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};

void setup() {
    for (int i = 0; i < 12; i++)
    {
        pinMode(digitalPins[i], INPUT);
    }

    for (int i = 0; i < 12; i++)
    {
        pinMode(analogPins[i], INPUT);
    }

    Serial.begin(9600);
}

void loop() {
/**/    
    Serial.print("\nAnalog:\t\t");

    for (int i = 0; i < 12; i++) {
        Serial.print("0");
        Serial.print(analogRead(analogPins[i]));
        Serial.print("\t");
    }
/**/
    Serial.print("\nDigital:\t");

    for (int i = 0; i < 12; i++) {
        Serial.print(digitalRead(digitalPins[i]));
        Serial.print("\t");
    }
/**/
    delay(100);
}