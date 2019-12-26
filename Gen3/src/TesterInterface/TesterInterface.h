#pragma once

#include <Arduino.h>
#include "../BluetoothInterface/BluetoothInterface.h"


#define TEST(s) \
    void setup() { \
        s##Setup();\
    } \
\
    void loop() { \
        s##Loop();\
    } 
    

class Debug {
    static bool usingBluetooth;
    static Bluetooth bluetooth;

    public:
    static void begin(int baud = 9600);

    static void useBluetooth();

    static void print(const char* );
    static void println(const char* );

    static void print(float);
    static void println(float);

    template <typename T>
    static void print(T v, int radix = 10) {
        if (usingBluetooth)
            bluetooth.print(v, radix);
        else
            Serial.print(v, radix);
    }

    template <typename T>
    static void println(T v, int radix = 10) {
        print(v, radix);
        print("\n");
    }
};
