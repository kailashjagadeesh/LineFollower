#include "pid_control.h"

PIDControl::PIDControl(uint16_t _targetValue, float *_kp, float *_kd, float *_ki)
{
    kp = *_kp;
    kd = *_kd;
    ki = *_ki;

    targetValue = _targetValue;

    errorSum = 0;
    lastError = 0;

    parameters[0] = 0.00;
    parameters[1] = 0.00;
    parameters[2] = 0.00;

    dp[0] = 1.00;
    dp[1] = 1.00;
    dp[2] = 1.00;
}

int16_t PIDControl::control(int16_t currentValue)
{
    int16_t c; //correction
    int16_t error = currentValue - targetValue;
    errorSum += error;

    c = kp * error + kd * (error - lastError) + ki * errorSum;

    lastError = error;
    return c;
}

void PIDControl::clear()
{
    errorSum = 0;
    lastError = 0;
}
float *PIDControl::tune()
{

    parameters[0] = kp;
    parameters[1] = kd;
    parameters[2] = ki;

    unsigned int best_err = follow_path(mode); // A function that returns 'error' must be added here
    unsigned int err;
    double sum = (dp[0] + dp[1] + dp[2]);

    while (sum > 0.000000001)
    {
        for (int i = 0; i < 3; i++)
        {
            parameters[i] += dp[i];
            err = follow_path(mode);

            if (err < best_err)
            {
                best_err = err;
                dp[i] *= 1.1;
            }
            else
            {
                parameters[i] -= 2 * dp[i];
                err = follow_path(mode);

                if (err < best_err)
                {
                    best_err = err;
                    dp[i] *= 1.1;
                }
                else
                {
                    parameters[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
        }
        sum = (dp[0] + dp[1] + dp[2]);
    }
}
