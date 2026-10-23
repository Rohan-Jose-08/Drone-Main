#include "include/pid.h"

static float clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

Pid::Pid(float kp, float ki, float kd, float outputLimit, float integralLimit, float derivativeCutoffHz)
    : kp(kp), ki(ki), kd(kd), outputLimit(outputLimit), integralLimit(integralLimit),
      derivativeTimeConstant(1.0f / (2.0f * 3.14159265f * derivativeCutoffHz))
{
}

float Pid::update(float setpoint, float measurement, float dt, bool holdIntegral)
{
    float error = setpoint - measurement;

    float derivative = 0;
    if (hasLastMeasurement && dt > 0)
    {
        derivative = -(measurement - lastMeasurement) / dt;
    }
    lastMeasurement = measurement;
    hasLastMeasurement = true;
    filteredDerivative += dt / (dt + derivativeTimeConstant) * (derivative - filteredDerivative);

    // Only keep the new integral if it doesn't push an already saturated output further.
    float newIntegral = clamp(integral + ki * error * dt, integralLimit);
    float output = kp * error + newIntegral + kd * filteredDerivative;
    bool pushingPastLimit = (output > outputLimit && error > 0) || (output < -outputLimit && error < 0);
    if (!pushingPastLimit && !holdIntegral)
    {
        integral = newIntegral;
    }

    return clamp(kp * error + integral + kd * filteredDerivative, outputLimit);
}

void Pid::reset()
{
    integral = 0;
    filteredDerivative = 0;
    hasLastMeasurement = false;
}
