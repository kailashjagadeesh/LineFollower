#ifndef LEDINTERFACE_H
#define LEDINTERFACE_H

class LED {
    public:
    int pins[2] = {14, 15};

    enum led {
        one, two
    };

    LED() {
        pinMode(pins[0], OUTPUT);
        pinMode(pins[1], OUTPUT);
    }

    void write(led l, int v) {
        digitalWrite(pins[l], v);
    }
} led;

#endif