#include "include/mixer.h"

// How much of each correction goes to each motor: roll, pitch, yaw.
static const float MIX[MOTOR_COUNT][3] = {
    {-1.0f, -1.0f, -1.0f},   // M1 rear right, clockwise
    {-1.0f,  1.0f,  1.0f},   // M2 front right, anticlockwise
    { 1.0f, -1.0f,  1.0f},   // M3 rear left, anticlockwise
    { 1.0f,  1.0f, -1.0f},   // M4 front left, clockwise
};

bool mix(float throttleUs, float roll, float pitch, float yaw, float minUs, float maxUs, float motorUs[MOTOR_COUNT])
{
    float delta[MOTOR_COUNT];
    float lowest = 0, highest = 0;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        delta[i] = roll * MIX[i][0] + pitch * MIX[i][1] + yaw * MIX[i][2];
        if (i == 0 || delta[i] < lowest)
        {
            lowest = delta[i];
        }
        if (i == 0 || delta[i] > highest)
        {
            highest = delta[i];
        }
    }

    float available = maxUs - minUs;
    bool saturated = highest - lowest > available;
    if (saturated)
    {
        float scale = available / (highest - lowest);
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            delta[i] *= scale;
        }
        lowest *= scale;
        highest *= scale;
    }

    // Shift the throttle just enough for every motor to fit.
    if (throttleUs + lowest < minUs)
    {
        throttleUs = minUs - lowest;
    }
    if (throttleUs + highest > maxUs)
    {
        throttleUs = maxUs - highest;
    }

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        float pulse = throttleUs + delta[i];
        motorUs[i] = pulse < minUs ? minUs : (pulse > maxUs ? maxUs : pulse);
    }
    return saturated;
}
