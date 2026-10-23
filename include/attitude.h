#ifndef ATTITUDE_H
#define ATTITUDE_H

// Tilt estimate: tracks which way is up, as a unit vector in the drone's axes (forward, right, down).
// The gyro turns the estimate as the drone rotates, and the accelerometer slowly pulls it toward the
// up direction it measures, which corrects gyro drift (a complementary filter, as in Mahony's).
// Yaw isn't estimated: without a magnetometer it can't be observed, and yaw is rate-controlled anyway.
class TiltEstimator
{
public:
    // kp: how fast the accelerometer pulls the estimate back, per second.
    // trustBand: the accelerometer is only used while it reads within this fraction of 1 g,
    // since otherwise the drone is accelerating and the reading isn't just gravity.
    TiltEstimator(float kp, float trustBand);

    // Starts from the accelerometer reading alone (in m/s^2, drone's axes); the drone should be still.
    // Returns false, leaving the estimator unready, if the reading is too far from 1 g to trust.
    bool reset(const float accel[3]);

    // ratesDps in deg/s and accel in m/s^2, both in the drone's axes.
    void update(const float ratesDps[3], const float accel[3], float dt);

    bool isReady() const { return ready; }
    bool accelInUse() const { return accelUsed; }    // whether the last reading was close enough to 1 g to use
    const float *up() const { return upVector; }

private:
    bool accelTrusted(const float accel[3], float &length) const;

    float kp, trustBand;
    float upVector[3] = {0, 0, -1};
    bool ready = false;
    bool accelUsed = false;
};

#endif
