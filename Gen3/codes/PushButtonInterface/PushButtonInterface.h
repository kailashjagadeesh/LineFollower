#ifndef PUSHBUTTON_INTERFACE_H
#define PUSHBUTTON_INTERFACE_H

#ifndef NUMBER_PUSHBUTTONS
#define NUMBER_PUSHBUTTONS 2
#define PUSHBUTTON_PINS {2, 50}
#endif

class PushButtonInterface {
    static const int *pins;
    static const int nPins;

    static bool initialized;
    public:

    void static init();
    void static waitForButton(int);
    static bool readState(int);
};

const int PushButtonInterface::nPins = NUMBER_PUSHBUTTONS;
const int* PushButtonInterface::pins = new int[NUMBER_PUSHBUTTONS] PUSHBUTTON_PINS; 
bool PushButtonInterface::initialized = false;

void PushButtonInterface::init() {
    for (int i = 0; i < nPins; i++) {
        pinMode(pins[i], INPUT_PULLUP);
    }

    initialized = true;
}

bool PushButtonInterface::readState(int pin) {
    if (!initialized) init();
    return !digitalRead(pins[pin]);
}

void PushButtonInterface::waitForButton(int pin) {
    if (!initialized) 
        init();
    while (digitalRead(pins[pin]));
    delay(1000);
}
#endif