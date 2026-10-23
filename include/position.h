#ifndef POSITION_H
#define POSITION_H

// Position and velocity along one horizontal axis (north or east), from the accelerometer and the GPS.
// The GPS velocity (measured from the satellites' Doppler shift) is precise but arrives only a few times a
// second, and describes where the drone was a little before it arrives; the accelerometer fills in between
// and reacts at once. Each fix is compared with what the estimate was when the fix was measured, so the lag
// doesn't drag the estimate back. Like the altitude estimate, it learns the accelerometer's offset, here
// from how it disagrees with the GPS velocity.
class PositionEstimator
{
public:
    // positionTimeConstant, velocityTimeConstant: roughly how many seconds the GPS position and velocity
    // take to pull the estimate back. maxAccel: horizontal accelerations beyond this (m/s^2) are taken as a
    // bad reading and ignored. maxOffset: the largest accelerometer offset (m/s^2) it will learn.
    // delayUpdates: how many update() calls old a fix is when it arrives (up to HISTORY - 1).
    PositionEstimator(float positionTimeConstant, float velocityTimeConstant, float maxAccel, float maxOffset,
                      int delayUpdates);

    // Starts at a GPS position (m from the origin) and velocity (m/s).
    void reset(float gpsPosition, float gpsVelocity);
    // Call with each new GPS fix.
    void setGps(float gpsPosition, float gpsVelocity);
    // Call with every IMU reading: acceleration along the axis in m/s^2. gpsFresh: whether the last GPS
    // fix is recent enough to steer by; while it isn't, the estimate runs on the accelerometer alone.
    void update(float accel, float dt, bool gpsFresh);

    bool isReady() const { return ready; }
    float position() const { return pos; }
    float velocity() const { return vel; }

    static const int HISTORY = 128;

private:
    float kp, kv, kb;
    float maxAccel, maxOffset;
    int delay;
    float posError = 0, velError = 0;       // from the last fix, against the estimate when it was measured
    float pos = 0, vel = 0, accelOffset = 0;
    float pastPos[HISTORY], pastVel[HISTORY];
    int newest = 0;
    bool ready = false;
};

// Rotates a vector from the drone's axes (forward, right, down) to north, east, down, using the tilt
// estimate's up vector and the heading (deg).
void body_to_ned(const float body[3], const float up[3], float headingDeg, float ned[3]);
// And back.
void ned_to_body(const float ned[3], const float up[3], float headingDeg, float body[3]);

#endif
