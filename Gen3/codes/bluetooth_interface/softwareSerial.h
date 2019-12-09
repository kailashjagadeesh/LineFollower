#include "soft_uart.h"

using namespace arduino_due;

// #define SERIAL_TC0_PIN 18        // TC0 software serial port's half duplex pin
// #define SERIAL_TC1_PIN 19        // TC1 software serial port's half duplex pin
// #define SOFT_UART_BIT_RATE 38400 // 38400 57600 38400 1200 19200 9600 115200
#define RX_BUF_LENGTH 256     // software serial port's reception buffer length
#define TX_BUF_LENGTH 256     // software serial port's transmision buffer length
#define RECEPTION_TIMEOUT 100 // milliseconds

// declaration of software serial port object serial_tc0
// which uses timer/counter channel TC0
serial_tc0_declaration(RX_BUF_LENGTH, TX_BUF_LENGTH);

// declaration of software serial port object serial_tc1
// which uses timer/counter channel TC1
serial_tc1_declaration(RX_BUF_LENGTH, TX_BUF_LENGTH);
#define receive_tc_definition
template <typename serial_tc_t>

class softwareSerial
{
    void receive_tc(serial_tc_t &serial_tc, unsigned long timeout);

public:
    softwareSerial();
    void println();
    char *read();
};
char *softwareSerial ::receive_tc(serial_tc_t &serial_tc, unsigned long timeout)
{
    int data = 0;
    unsigned long last_time = millis();
    char outputString[25] = "";
    int iteration_number = 0;
    do
    {
        if (serial_tc.available())
        {
            if ((data = serial_tc.read()) >= 0)
            {
                last_time = millis();
                // Serial.print(static_cast<char>(data));
                outputString[iteration_number++] = static_cast<char>(data);
            }
            else
            {
                if (serial_tc.bad_status())
                {
                    // last_time = millis();
                    // Serial.print("[");
                    // if (serial_tc.bad_start_bit())
                    //     Serial.print("BAD_START_BIT,");
                    // if (serial_tc.bad_parity())
                    //     Serial.print("BAD_PARITY,");
                    // if (serial_tc.bad_stop_bit())
                    //     Serial.print("BAD_STOP_BIT,");
                    // Serial.print((serial_tc.get_last_data_status() >> 16), BIN);
                    // Serial.print("]");
                }
            }
        }
    } while (millis() - last_time < timeout);
    return outputString;
}
//(RX pin,TX pin , baudRate)
softwareSerial::softwareSerial(int rxPin, int txPin, int baudRate)
{
    receive_tc_definition;
    serial_tc0.half_duplex_begin(
        txPin,
        baudRate,
        soft_uart::parity_codes::EVEN_PARITY,
        soft_uart::stop_bit_codes::ONE_STOP_BIT,
        false // on transmission mode (the default is on reception mode)
    );
    serial_tc1.half_duplex_begin(
        rxPin,
        baudRate,
        soft_uart::data_bit_codes::EIGHT_BITS,
        soft_uart::parity_codes::EVEN_PARITY,
        soft_uart::stop_bit_codes::ONE_STOP_BIT
        // initially on reception mode, last argument is true by default
    );
}
// Pass a string to send via BT
void softwareSerial ::println(char *string)
{
    serial_tc0.println(string);
}

// Returns a char array, of the read message
char *softwareSerial ::read()
{
    unsigned long timeout = static_cast<unsigned long>(2 * 1000 * serial_tc1.get_frame_time());
    if (timeout < RECEPTION_TIMEOUT)
        timeout = RECEPTION_TIMEOUT;

    receive_tc(serial_tc1, timeout);
}