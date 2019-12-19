
#include <soft_uart.h>
#include <string.h>

using namespace arduino_due;

#define SERIAL_TC0_PIN 19       // TC0 software serial port's half duplex pin
#define SERIAL_TC1_PIN 18       // TC1 software serial port's half duplex pin
#define SOFT_UART_BIT_RATE 9600 // 38400 57600 38400 1200 19200 9600 115200 115200 75
#define RX_BUF_LENGTH 256       // software serial port's reception buffer length
#define TX_BUF_LENGTH 256       // software serial port's transmision buffer length
#define TX_INACTIVE_TIME 50     // milliseconds
#define RECEPTION_TIMEOUT 100   // milliseconds

// declaration of software serial port object serial_tc0
// which uses timer/counter channel TC0
serial_tc0_declaration(RX_BUF_LENGTH, TX_BUF_LENGTH);

// declaration of software serial port object serial_tc1
// which uses timer/counter channel TC1
serial_tc1_declaration(RX_BUF_LENGTH, TX_BUF_LENGTH);

// FIX: function template receive_tc is defined in
// #define to avoid it to be considered a function
// prototype when integrating all .ino files in one
// whole .cpp file. Without this trick the compiler
// complains about the definition of the template
// function.
#define receive_tc_definition \
  template <typename serial_tc_t>

// FIX: here we instantiate the template definition
// of receive_tc
class Bluetooth
{
public:
  void begin();
  void print(uint32_t, int radix = 10);
  void println(uint32_t, int radix = 10);
  void print(float);
  void println(float);
  void print(const char *);
  void println(const char *);
};
void Bluetooth::begin()
{

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

void Bluetooth ::print(float number)
{

  char temp[15];

  sprintf(temp, "%f", number);
  serial_tc0.write(temp);
}

void Bluetooth ::println(float number)
{

  char temp[15];

  sprintf(temp, "%f\n", number);
  serial_tc0.write(temp);
}

void Bluetooth ::print(uint32_t number, int radix)
{

  char temp[15];
  switch(radix) {
    case 10: sprintf(temp, "%d", number); break;
    case 16: sprintf(temp, "%x", number); break;
    case 2: 
    char *p;
    int i;
    p = temp;

    for (i = 8; i >= 0; i--)
        *(p++) = (number & (1 << i))? '1' : '0';
    *p = 0;
    break;
  }
  
  serial_tc0.write(temp);
}
void Bluetooth ::println(uint32_t number, int radix)
{
  print(number, radix);
  serial_tc0.write("\n");
}
//receive_tc_definition;
void Bluetooth ::print(const char *array)
{
  serial_tc0.write(array);
}
void Bluetooth ::println(const char *array)
{
  serial_tc0.write(array);
  serial_tc0.write("\n");
}