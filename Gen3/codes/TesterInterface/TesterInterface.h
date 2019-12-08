#ifndef TESTER_INTERFACE_H
#define TESTER_INTERFACE_H

#define TEST(s, l) \
    void setup() { \
        runTest(s, sizeof(s)/sizeof(void*)); \
    } \
\
    void loop() { \
        runTest(l, sizeof(l)/sizeof(void*)); \
    } 


typedef void (*testFunction) (void);

void runTest(testFunction funcs[], int n) {
    for (int i = 0; i < n; i++)
        funcs[i]();
}

#endif