#include "PushButtonInterface.h"

#include <Arduino.h>
#include "../configure.h"

#ifndef NUMBER_PUSHBUTTONS
#define NUMBER_PUSHBUTTONS 2
#endif 

#ifndef PUSHBUTTON_PINS
#define PUSHBUTTON_PINS {50, 2}
#endif

const int PushButtonInterface::nPins = NUMBER_PUSHBUTTONS;
const int* PushButtonInterface::pins = new int[NUMBER_PUSHBUTTONS] PUSHBUTTON_PINS; 
bool PushButtonInterface::initialized = false;

/*
    will initialize @NUMBER_PUSHBUTTONS push buttons
    connected to @PUSHBUTTON_PINS pins
*/
void PushButtonInterface::init() {
    for (int i = 0; i < nPins; i++) {
        pinMode(pins[i], INPUT_PULLUP);
    }

    initialized = true;
}

bool PushButtonInterface::read(int pin) {
    if (!initialized) init();
    return !digitalRead(pins[pin]);
}

void PushButtonInterface::waitForButton(int pin) {
    if (!initialized) 
        init();
    while (!read(pin));
    delay(1000);
}