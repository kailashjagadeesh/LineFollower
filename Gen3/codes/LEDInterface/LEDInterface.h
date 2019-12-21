#ifndef LEDINTERFACE_H
#define LEDINTERFACE_H

#include "TesterInterface.h"

#ifndef NUM_LED
    //number of LED pins (LEDInterface)
    #define NUM_LED 2
    //Array of leds (LEDInterface)
    #define LED_PINS {14, 15}
#endif

class LED {
    static bool initialized;
    public:
    static const int * const pins;
    static const int nPins;

    //initialize LED pins
    static void init();
    //write an LED pin
    static void write(int i, int v);
};

const int * const LED::pins = new int[NUM_LED] LED_PINS;
const int LED::nPins = NUM_LED;
bool LED::initialized = false;

void LED::init() {
    for (int i = 0; i < nPins; ++i)
        pinMode(pins[i], OUTPUT);

    initialized = true;
}

void LED::write(int i, int v) {
    if (initialized)
        digitalWrite(pins[i], v);
    else 
        init();
}

#endif