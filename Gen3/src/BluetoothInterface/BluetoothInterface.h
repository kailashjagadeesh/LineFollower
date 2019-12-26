#ifndef BLUETOOTH_INTERFACE_H
#define BLUETOOTH_INTERFACE_H

#include <stdint.h>

class Bluetooth
{
public:
  void begin();
  void print(uint32_t, int radix = 10);
  void println(uint32_t, int radix = 10);
  void print(uint16_t, int radix = 10);
  void println(uint16_t, int radix = 10);
  void print(int, int radix = 10);
  void println(int, int radix = 10);
  void print(float);
  void println(float);
  void print(const char *);
  void println(const char *);
};

#endif