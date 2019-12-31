#include "Twiddle.h"
#include "../configure.h"
#include "../LEDInterface/LEDInterface.h"
#include "../TesterInterface/TesterInterface.h"

#include <Arduino.h>

void Twiddle::autoTune(MotorPIDControl& pid, Sensors &sensors) {
    float best_err = abs(pid.getError(sensors.readLine()));
    float err;

    double sum = (pid.dp[0] + pid.dp[1]);
    while (sum > PID_TUNING_ACCURACY) {
        Debug::print((float)(pid.parameters[0]));
        Debug::print(" |-1| ");
        Debug::println((float)(pid.parameters[1]));
        for (int i = 0; i < 2; i++) {
            pid.parameters[i] += pid.dp[i];
            Debug::print((float)(pid.parameters[0]));
            Debug::print(" |0| ");
            Debug::println((float)(pid.parameters[1]));
            pid.setSpeedBasedOnCorrection(pid.control(sensors.readLine()));
            err = abs(pid.getError(sensors.readLine()));
            if (err < best_err) {
                best_err = err;
                pid.dp[i] *= 1.1;
            }
            else {
                pid.parameters[i] -= 2 * pid.dp[i];
                pid.setSpeedBasedOnCorrection(pid.control(sensors.readLine()));
                err = abs(pid.getError(sensors.readLine()));
                Debug::print((float)(pid.parameters[0]));
                Debug::print(" |1| ");
                Debug::println((float)(pid.parameters[1]));

                if (err < best_err) {
                    best_err = err;
                    pid.dp[i] *= 1.1;
                }
                else {
                    pid.parameters[i] += pid.dp[i];
                    Debug::print((float)(pid.parameters[0]));
                    Debug::print(" |2| ");
                    Debug::println((float)(pid.parameters[1]));
                    pid.dp[i] *= 0.9;
                }
            }
        }
        sum = (pid.dp[0] + pid.dp[1] + pid.dp[2]);
    }
}