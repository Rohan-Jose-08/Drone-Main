#ifndef ALTITUDE_H
#define ALTITUDE_H

// Altitude and climb rate estimate. Integrating the accelerometer's vertical acceleration follows quick
// changes but drifts away within seconds; the barometer stays at the right height but on its own is slow
// and noisy. This blends the two (a third-order complementary filter), and learns the accelerometer's
// offset from how they disagree.
class AltitudeEstimator
{
public:
    // timeConstant: roughly how many seconds the estimate rides on the accelerometer before the barometer
    // pulls it back. Longer smooths barometer noise more, but lets accelerometer errors run on longer.
    // maxAccel: vertical accelerations beyond this (m/s^2) are taken as a faulty reading or a tumble and
    // ignored, so they can't throw the estimate off for long afterwards.
    // maxOffset: the largest accelerometer offset (m/s^2) it will learn.
    AltitudeEstimator(float timeConstant, float maxAccel, float maxOffset);

    // Starts at a barometer altitude (m), not climbing.
    void reset(float baroAltitude);
    // Call with each new barometer altitude (m).
    void setBaro(float baroAltitude);
    // Call with every IMU reading: vertical acceleration in m/s^2, up positive, with gravity taken out.
    // baroFresh: whether the last barometer reading is recent enough to steer by; while it isn't, the
    // estimate runs on the accelerometer alone.
    void update(float verticalAccel, float dt, bool baroFresh);

    bool isReady() const { return ready; }
    float altitude() const { return height; }
    float climbRate() const { return climb; }       // m/s, up positive

private:
    float k1, k2, k3;
    float maxAccel, maxOffset;
    float baroHeight = 0, height = 0, climb = 0, accelOffset = 0;
    bool ready = false;
};

#endif
