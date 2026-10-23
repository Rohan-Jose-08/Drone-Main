#include <math.h>
#include "include/altitude.h"

// These gains make the height, climb rate and offset errors all die away at the same rate, 1/timeConstant.
AltitudeEstimator::AltitudeEstimator(float timeConstant, float maxAccel, float maxOffset)
    : k1(3.0f / timeConstant), k2(3.0f / (timeConstant * timeConstant)),
      k3(1.0f / (timeConstant * timeConstant * timeConstant)), maxAccel(maxAccel), maxOffset(maxOffset)
{
}

void AltitudeEstimator::reset(float baroAltitude)
{
    baroHeight = height = baroAltitude;
    climb = 0;
    accelOffset = 0;
    ready = true;
}

void AltitudeEstimator::setBaro(float baroAltitude)
{
    baroHeight = baroAltitude;
}

void AltitudeEstimator::update(float verticalAccel, float dt, bool baroFresh)
{
    // The gap to the latest barometer reading pulls on the height, the climb rate and the offset. While the
    // accelerometer reading is implausible, the barometer carries the estimate alone and the offset is left as is.
    float error = baroFresh ? baroHeight - height : 0;
    bool accelOk = fabsf(verticalAccel) < maxAccel;
    if (accelOk)
    {
        accelOffset = fminf(fmaxf(accelOffset + k3 * error * dt, -maxOffset), maxOffset);
    }
    float accel = accelOk ? verticalAccel + accelOffset : 0;
    height += (climb + k1 * error) * dt + 0.5f * accel * dt * dt;
    climb += (accel + k2 * error) * dt;
}
