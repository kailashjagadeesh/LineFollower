#ifndef PUSHBUTTON_INTERFACE_H
#define PUSHBUTTON_INTERFACE_H

class PushButtonInterface {
    static const int *pins;
    static const int nPins;

    static bool initialized;
    public:

    void static init();
    void static waitForButton(int);
    static bool read(int);
};

#endif