void setup() {
    pinMode(A9, INPUT);
    pinMode(A10, INPUT);
    Serial.begin(9600);
}

void loop() {
    Serial.print(analogRead(A9));
    Serial.print("\t");
    Serial.println(analogRead(A10));
}