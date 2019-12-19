#ifndef TESTER_INTERFACE_H
#define TESTER_INTERFACE_H

#ifndef SERIALD 
//Serial to print debug info (TesterInterface)
#define SERIALD Serial
#endif

#ifndef SERIALERR
//Serial to print error info (TesterInterface)
#define SERIALERR Serial
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