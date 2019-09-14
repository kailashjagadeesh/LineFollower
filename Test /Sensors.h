#include <QTRSensorsAnalog.h>
#include <EEPROM.h>

#define NUM_SENSORS 8                  // number of sensors used
#define TIMEOUT 4                      // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN QTR_NO_EMITTER_PIN // emitter is not controlled
#define database_address_calibration 0

QTRSensorsAnalog qtra((unsigned char[]){0, 1, 2, 3, 4, 5, 6, 7}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

struct calib_data
{
    unsigned int Min[8], Max[8];
} cal;

class eeprom
{
    int check_data_inequality(calib_data Old, calib_data New)
    {
        for (int i = 0; i < 8; i++)
        {
            if ((Old.Min[i] != New.Min[i]) || (Old.Max[i] != New.Max[i]))
                return 1;
        }
        return 0;
    }
public:
    calib_data read(int address)
    {
        calib_data temp_obj;
        EEPROM.get(address, temp_obj);
        return temp_obj;
    }

    void write(int address, calib_data data)
    {
        if (check_data_inequality(read(address), data))
        {
            EEPROM.put(address, data);
        }
    }
};
eeprom database;

void calibrate_sensor()
{
    pinMode(12, INPUT);
    pinMode(13, OUTPUT);
    if (digitalRead(12) == HIGH)
    {
        Serial.println("\nCallibrating");
        for (int i = 0; i < 300; i++)
        {
            qtra.calibrate();
            delay(20);
        }

        for (int i = 0; i < 8; i++)
        {
            cal.Min[i] = qtra.calibratedMinimumOn[i];
            cal.Max[i] = qtra.calibratedMaximumOn[i];
        }
        database.write(database_address_calibration, cal);
        delay(50);
        Serial.println("Calibrated");
    }
    else
    {
        Serial.println("Callibrated values");
        qtra.calibrate();
        cal = database.read(database_address_calibration);
        for (int i = 0; i < 8; i++)
        {
            Serial.print(cal.Min[i]);
            Serial.print('\t');
            qtra.calibratedMinimumOn[i] = cal.Min[i];
        }
        Serial.println();
        for (int i = 0; i < 8; i++)
        {
            Serial.print(cal.Max[i]);
            Serial.print('\t');
            qtra.calibratedMaximumOn[i] = cal.Max[i];
        }
        Serial.println();
    }
    digitalWrite(13, HIGH);
}
