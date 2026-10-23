#include <math.h>
#include "include/filters.h"

// Coefficients from the Audio EQ Cookbook (R. Bristow-Johnson).

Biquad Biquad::lowpass(float cutoffHz, float sampleHz)
{
    Biquad filter;
    if (cutoffHz <= 0 || cutoffHz >= sampleHz / 2)
    {
        return filter;
    }
    float w0 = 2.0f * 3.14159265f * cutoffHz / sampleHz;
    float alpha = sinf(w0) / (2.0f * 0.70710678f);
    float a0 = 1.0f + alpha;
    filter.b0 = (1.0f - cosf(w0)) / 2.0f / a0;
    filter.b1 = (1.0f - cosf(w0)) / a0;
    filter.b2 = filter.b0;
    filter.a1 = -2.0f * cosf(w0) / a0;
    filter.a2 = (1.0f - alpha) / a0;
    return filter;
}

Biquad Biquad::notch(float centreHz, float q, float sampleHz)
{
    Biquad filter;
    if (centreHz <= 0 || centreHz >= sampleHz / 2)
    {
        return filter;
    }
    float w0 = 2.0f * 3.14159265f * centreHz / sampleHz;
    float alpha = sinf(w0) / (2.0f * q);
    float a0 = 1.0f + alpha;
    filter.b0 = 1.0f / a0;
    filter.b1 = -2.0f * cosf(w0) / a0;
    filter.b2 = filter.b0;
    filter.a1 = filter.b1;
    filter.a2 = (1.0f - alpha) / a0;
    return filter;
}

float Biquad::apply(float input)
{
    float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    return output;
}
