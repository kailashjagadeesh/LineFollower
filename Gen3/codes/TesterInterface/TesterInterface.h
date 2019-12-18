#ifndef TESTER_INTERFACE_H
#define TESTER_INTERFACE_H

#ifndef SERIALD 
//Serial to print debug/error info (TesterInterface)
#define SERIALD Serial
#endif

#define TEST(s) \
    void setup() { \
        s##Setup();\
    } \
\
    void loop() { \
        s##Loop();\
    } 
    
#endif