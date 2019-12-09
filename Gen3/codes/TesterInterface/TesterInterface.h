#ifndef TESTER_INTERFACE_H
#define TESTER_INTERFACE_H

#define TEST(s) \
    void setup() { \
        s##Setup();\
    } \
\
    void loop() { \
        s##Loop();\
    } 
    
#endif