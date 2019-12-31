#include "TesterInterface.h"

#include <Arduino.h>
bool Debug::usingBluetooth;
Bluetooth Debug::bluetooth;

void Debug::begin(int baud) {
    Serial.begin(baud);
    usingBluetooth = false;
}

void Debug::useBluetooth() {
    usingBluetooth = true;
    bluetooth.begin();
}

void Debug::print(const char* v) {
    if (usingBluetooth)
        bluetooth.print(v);
    else
        Serial.print(v);
}

void Debug::println(const char* v) {
    print(v);
    print("\n");
}

void Debug::print(char* v) {
    if (usingBluetooth)
        bluetooth.print((const char*)v);
    else
        Serial.print(v);
}

void Debug::println(char* v) {
    print((const char*)v);
    print("\n");
}

void Debug::print(float v) {
    if (usingBluetooth)
        bluetooth.print(v);
    else
        Serial.print(v);
}

void Debug::println(float v) {
    print(v);
    print("\n");
}