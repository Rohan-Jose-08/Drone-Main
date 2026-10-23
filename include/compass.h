#ifndef COMPASS_H
#define COMPASS_H

// Heading (deg, 0 = magnetic north, clockwise seen from above) from a magnetometer reading in the drone's
// axes (forward, right, down), with the tilt taken out using the up vector from the tilt estimate.
float compass_heading(const float mag[3], const float up[3]);

// Hard-iron calibration: turned through every orientation, the midpoint of the readings on each axis is
// the offset to subtract from that axis.
struct CompassCalibration
{
    float lowest[3] = {0, 0, 0}, highest[3] = {0, 0, 0};
    bool started = false;

    void add(const float reading[3]);
    void offsets(float out[3]) const;
};

#endif
