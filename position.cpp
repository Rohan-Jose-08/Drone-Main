#include <math.h>
#include "include/position.h"

#define DEG_TO_RAD 0.0174532925f

// The velocity and offset errors die away together (critically damped) at 1/velocityTimeConstant, and the
// position error at 1/positionTimeConstant.
PositionEstimator::PositionEstimator(float positionTimeConstant, float velocityTimeConstant, float maxAccel,
                                     float maxOffset, int delayUpdates)
    : kp(1.0f / positionTimeConstant), kv(2.0f / velocityTimeConstant),
      kb(1.0f / (velocityTimeConstant * velocityTimeConstant)), maxAccel(maxAccel), maxOffset(maxOffset),
      delay(delayUpdates < HISTORY ? delayUpdates : HISTORY - 1)
{
}

void PositionEstimator::reset(float gpsPosition, float gpsVelocity)
{
    pos = gpsPosition;
    vel = gpsVelocity;
    for (int i = 0; i < HISTORY; i++)
    {
        pastPos[i] = pos;
        pastVel[i] = vel;
    }
    posError = velError = 0;
    accelOffset = 0;
    ready = true;
}

void PositionEstimator::setGps(float gpsPosition, float gpsVelocity)
{
    int then = (newest - delay + HISTORY) % HISTORY;
    posError = gpsPosition - pastPos[then];
    velError = gpsVelocity - pastVel[then];
}

void PositionEstimator::update(float accel, float dt, bool gpsFresh)
{
    // The last fix's errors steer the estimate until the next fix replaces them.
    float posCorrection = gpsFresh ? posError : 0;
    float velCorrection = gpsFresh ? velError : 0;
    bool accelOk = fabsf(accel) < maxAccel;
    if (accelOk)
    {
        accelOffset = fminf(fmaxf(accelOffset + kb * velCorrection * dt, -maxOffset), maxOffset);
    }
    float a = accelOk ? accel + accelOffset : 0;
    pos += (vel + kp * posCorrection) * dt + 0.5f * a * dt * dt;
    vel += (a + kv * velCorrection) * dt;

    newest = (newest + 1) % HISTORY;
    pastPos[newest] = pos;
    pastVel[newest] = vel;
}

void body_to_ned(const float body[3], const float up[3], float headingDeg, float ned[3])
{
    float roll = atan2f(-up[1], -up[2]);
    float pitch = atan2f(up[0], sqrtf(up[1] * up[1] + up[2] * up[2]));
    float yaw = headingDeg * DEG_TO_RAD;
    float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch), cy = cosf(yaw), sy = sinf(yaw);
    // Yaw, then pitch, then roll (the usual aerospace order).
    ned[0] = cy * cp * body[0] + (cy * sp * sr - sy * cr) * body[1] + (cy * sp * cr + sy * sr) * body[2];
    ned[1] = sy * cp * body[0] + (sy * sp * sr + cy * cr) * body[1] + (sy * sp * cr - cy * sr) * body[2];
    ned[2] = -sp * body[0] + cp * sr * body[1] + cp * cr * body[2];
}

void ned_to_body(const float ned[3], const float up[3], float headingDeg, float body[3])
{
    // The rotation's transpose: each axis of the drone, written in north, east, down, dotted with the vector.
    float axes[3][3], unit[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < 3; i++)
    {
        body_to_ned(unit[i], up, headingDeg, axes[i]);
        body[i] = axes[i][0] * ned[0] + axes[i][1] * ned[1] + axes[i][2] * ned[2];
    }
}
