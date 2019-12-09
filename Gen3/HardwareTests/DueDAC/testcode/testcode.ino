void setup() {
    pinMode(DAC1, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    for (int i = 0; i <= 10; i++) {
        analogWriteResolution(8);
        analogWrite(DAC1, i * 255 / 10);
        Serial.print("DAC1: ");
        Serial.println(i * 255 / 10);
        Serial.print("Expected voltage: ");
        Serial.println(i * 3.3 / 10);
        delay(3000);
    }
}