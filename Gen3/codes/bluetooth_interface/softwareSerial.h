#include <soft_uart.h>
#include <fifo.h>
using namespace arduino_due;


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
receive_tc_definition; 

class softSerial
{
    

public:
    softSerial(int,int,int);
    void println(char *);

};

//(RX pin,TX pin , baudRate)
softSerial::softSerial(int rxPin, int txPin, int baudRate)
{
    
    serial_tc0.half_duplex_begin(
        txPin,
        baudRate,
        soft_uart::data_bit_codes::NINE_BITS,
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
void softSerial ::println(char *string)
{
    serial_tc0.println(string);
}
