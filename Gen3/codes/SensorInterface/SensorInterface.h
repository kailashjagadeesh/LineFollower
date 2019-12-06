#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#define NUM_SENSORS 9
#define CLASSIFICATION_THRESHOLD 511

class Sensors
{
    //ds to strore the readings
    uint16_t analogReadings[12];
    uint16_t digitalReadings[12];
    uint16_t calibratedHighValues[12];
    uint16_t calibratedLowValues[12];

    //pin definitions
    static const uint8_t analogPins[12];
    static const uint8_t digitalPins[12];

public:
    //read values
    void readSensorsAnalog();
    void readSensorsDigital();
    void readSensors();

    //callibration (set DAC Values)
    void calibrate();

    //get line for PID
    uint16_t readLine();

    Sensors();

    //debug info
    void printDebugInfo();
    void printSensorReadings();
};

//pin definitions
const uint8_t Sensors::analogPins[12] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
const uint8_t Sensors::digitalPins[12] = {6, 5, 4, 3, 40, 28, 30, 26, 32, 38, 34, 36};

//debug
void Sensors::printDebugInfo()
{
    Serial.println("SENSOR CALIBRATED VALUES [HIGH]:");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(calibratedHighValues[i]);
        Serial.print("\t");
    }

    Serial.println("\n\nSENSOR CALIBRATED VALUES [LOW]:");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(calibratedLowValues[i]);
        Serial.print("\t");
    }
}

void Sensors::printSensorReadings()
{
    Serial.print("\n\nSENSOR READINGS [Analog]: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(analogReadings[i]);
        Serial.print("\t");
    }

    Serial.print("\n\nSENSOR READINGS [Digital]: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(digitalReadings[i]);
        Serial.print("\t");
    }
}

//calibration of the DAC value
void Sensors::calibrate()
{
    uint8_t tempH = 0, tempL = 0;
    //read analog values over 100 times
    for (int i = 0; i < 100; i++)
    {
        readSensorsAnalog();

        for (int j = 0; i < NUM_SENSORS; j++)
        {
            if (analogReadings[j] >= CLASSIFICATION_THRESHOLD)
            {
                calibratedHighValues[j] += analogReadings[j];
                tempH++;
            }
            else
            {
                calibratedLowValues[j] += analogReadings[j];
                tempL++;
            }
        }

        //delay between readings
        delay(10);
    }

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        calibratedHighValues[i] /= tempH;
        calibratedLowValues[i] /= tempL;
    }

    //setting the DAC values of the average of the average of high and low means
    uint16_t temp = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        temp += calibratedHighValues[i] + calibratedLowValues[i];
    }

    analogWrite(DAC0, temp / NUM_SENSORS);
}

Sensors::Sensors()
{
    //initialize pins
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pinMode(digitalPins[i], INPUT);
        pinMode(analogPins[i], INPUT);

        calibratedHighValues[i] = 0;
        calibratedLowValues[i] = 0;
    }
}

void Sensors::readSensorsAnalog()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        analogReadings[i] = analogRead(analogPins[i]);
    }
}

void Sensors::readSensorsDigital()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        digitalReadings[i] = digitalRead(digitalPins[i]);
    }
}

void Sensors::readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        digitalReadings[i] = digitalRead(digitalPins[i]);
        analogReadings[i] = analogRead(analogPins[i]);
    }
}

uint16_t Sensors::readLine()
{
    //TODO
}

#endif