#ifndef FILTERS_H
#define FILTERS_H

// Second-order (biquad) filter. A frequency of 0, or one at or above half the sample rate,
// gives a filter that passes the signal through unchanged.
class Biquad
{
public:
    static Biquad lowpass(float cutoffHz, float sampleHz);        // Butterworth
    static Biquad notch(float centreHz, float q, float sampleHz);  // higher q is narrower

    float apply(float input);

private:
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
};

#endif
