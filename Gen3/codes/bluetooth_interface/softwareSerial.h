
#include <soft_uart.h>

using namespace arduino_due;

#define SERIAL_TC0_PIN 18 // TC0 software serial port's half duplex pin
#define SERIAL_TC1_PIN 19 // TC1 software serial port's half duplex pin
#define SOFT_UART_BIT_RATE 9600 // 38400 57600 38400 1200 19200 9600 115200 115200 75
#define RX_BUF_LENGTH 256 // software serial port's reception buffer length
#define TX_BUF_LENGTH 256 // software serial port's transmision buffer length
#define TX_INACTIVE_TIME 50 // milliseconds
#define RECEPTION_TIMEOUT 100 // milliseconds


// declaration of software serial port object serial_tc0
// which uses timer/counter channel TC0
serial_tc0_declaration(RX_BUF_LENGTH,TX_BUF_LENGTH);

// declaration of software serial port object serial_tc1
// which uses timer/counter channel TC1
serial_tc1_declaration(RX_BUF_LENGTH,TX_BUF_LENGTH);

// FIX: function template receive_tc is defined in
// #define to avoid it to be considered a function
// prototype when integrating all .ino files in one
// whole .cpp file. Without this trick the compiler
// complains about the definition of the template
// function.
#define receive_tc_definition \
template<typename serial_tc_t> \


// FIX: here we instantiate the template definition
// of receive_tc


void bluetoothInit() {

    serial_tc0.half_duplex_begin(
    SERIAL_TC0_PIN, 
    SOFT_UART_BIT_RATE,
    soft_uart::data_bit_codes::EIGHT_BITS,
    soft_uart::parity_codes::NO_PARITY,
    soft_uart::stop_bit_codes::ONE_STOP_BIT,
    false // on transmission mode (the default is on reception mode) 
  );
  serial_tc1.half_duplex_begin(
    SERIAL_TC1_PIN, 
    SOFT_UART_BIT_RATE,
    soft_uart::data_bit_codes::EIGHT_BITS,
    soft_uart::parity_codes::NO_PARITY,
    soft_uart::stop_bit_codes::ONE_STOP_BIT
    // initially on reception mode, last argument is true by default
  );   

}
//receive_tc_definition;
void bluetoothPrint(uint32_t counter) {


  serial_tc0.write(counter);


}
