class Ultrasonic
{
    float time;
    float distance;
    int trigPin, echoPin;

public:
    Ultrasonic(int, int);
    float measureDistance();
    bool detectBlock(int thresholdDistance);
};
// Initializes GPIOs for using ultrasonic
Ultrasonic ::Ultrasonic(int trig = 24, int echo = 22)
{
    trigPin = trig;
    echoPin = echo;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
}
// Returns distance measured by the sensor in CMs
float Ultrasonic ::measureDistance()
{
    digitalWrite(trigPin, HIGH);

    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);

    time = (float)pulseIn(echoPin, HIGH);

    distance = (time * 0.034) / 2;
    return distance;
}
//Argument holds threshold distance for deciding if block is present
bool Ultrasonic :: detectBlock(int thresholdDistance = 5)
{
    return ( measureDistance() <= thresholdDistance) ? (1) : (0);
}