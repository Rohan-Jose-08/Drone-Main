#include <math.h>
#include "include/attitude.h"

#define GRAVITY_MS2 9.80665f
#define DEG_TO_RAD 0.0174532925f

static void cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

TiltEstimator::TiltEstimator(float kp, float trustBand) : kp(kp), trustBand(trustBand)
{
}

bool TiltEstimator::accelTrusted(const float accel[3], float &length) const
{
    length = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    return fabsf(length - GRAVITY_MS2) < trustBand * GRAVITY_MS2;
}

bool TiltEstimator::reset(const float accel[3])
{
    float length;
    if (!accelTrusted(accel, length))
    {
        return false;
    }
    for (int i = 0; i < 3; i++)
    {
        upVector[i] = accel[i] / length;
    }
    ready = true;
    accelUsed = true;
    return true;
}

void TiltEstimator::update(const float ratesDps[3], const float accel[3], float dt)
{
    float rates[3] = {ratesDps[0] * DEG_TO_RAD, ratesDps[1] * DEG_TO_RAD, ratesDps[2] * DEG_TO_RAD};

    // Correction: an extra rotation that turns the estimate toward the measured up direction.
    float length;
    accelUsed = accelTrusted(accel, length);
    if (accelUsed)
    {
        float measured[3] = {accel[0] / length, accel[1] / length, accel[2] / length};
        float error[3];
        cross(measured, upVector, error);
        for (int i = 0; i < 3; i++)
        {
            rates[i] += kp * error[i];
        }
    }

    // A direction fixed in the world turns the opposite way to the drone: d(up)/dt = up x rates.
    float change[3];
    cross(upVector, rates, change);
    float norm = 0;
    for (int i = 0; i < 3; i++)
    {
        upVector[i] += change[i] * dt;
        norm += upVector[i] * upVector[i];
    }
    norm = sqrtf(norm);
    for (int i = 0; i < 3; i++)
    {
        upVector[i] /= norm;
    }
}
