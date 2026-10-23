#include <math.h>
#include "include/compass.h"

#define RAD_TO_DEG 57.2957795f

float compass_heading(const float mag[3], const float up[3])
{
    // Turn the reading back to level (undo roll, then pitch), leaving only the rotation about the vertical.
    float roll = atan2f(-up[1], -up[2]);
    float pitch = atan2f(up[0], sqrtf(up[1] * up[1] + up[2] * up[2]));
    float levelForward = mag[0] * cosf(pitch) + (mag[1] * sinf(roll) + mag[2] * cosf(roll)) * sinf(pitch);
    float levelRight = mag[1] * cosf(roll) - mag[2] * sinf(roll);
    return atan2f(-levelRight, levelForward) * RAD_TO_DEG;
}

void CompassCalibration::add(const float reading[3])
{
    for (int axis = 0; axis < 3; axis++)
    {
        if (!started || reading[axis] < lowest[axis])
        {
            lowest[axis] = reading[axis];
        }
        if (!started || reading[axis] > highest[axis])
        {
            highest[axis] = reading[axis];
        }
    }
    started = true;
}

void CompassCalibration::offsets(float out[3]) const
{
    for (int axis = 0; axis < 3; axis++)
    {
        out[axis] = (lowest[axis] + highest[axis]) / 2;
    }
}
