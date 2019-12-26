#ifndef LEDINTERFACE_H
#define LEDINTERFACE_H

class LED {
    static bool initialized;
    public:
    static const int * const pins;
    static const int nPins;

    //initialize LED pins
    static void init();
    //write an LED pin
    static void write(int i, int v);
    static void toggle(int i);
};

#endif