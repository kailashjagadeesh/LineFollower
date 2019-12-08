volatile uint8_t backSensorState = 0; // Contains state of all sensors except the front extreme one at any time
volatile bool CFState = 0;            // 1-Means a white line under sensor
enum sensorIndex
{
    RB2 = 0,
    RB1,
    RM,
    RF,
    LF,
    LM,
    LB1,
    LB2
};
void CFISR()
{
    CFState = !CFState;
}
void LFISR()
{
    backSensorState ^= (1 << LF);
}
void LMISR()
{
    backSensorState ^= (1 << LM);
}
void LB1ISR()
{
    backSensorState ^= (1 << LB1);
}
void LB2ISR()
{
    backSensorState ^= (1 << LB2);
}

void RFISR()
{
    backSensorState ^= (1 << LF);
}
void RMISR()
{
    backSensorState ^= (1 << LM);
}
void RB1ISR()
{
    backSensorState ^= (1 << LB1);
}
void RB2ISR()
{
    backSensorState ^= (1 << LB2);
}