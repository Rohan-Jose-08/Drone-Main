#ifndef PID_H
#define PID_H

// One-axis PID controller. The caller passes the time step on every update, so it works at any loop rate.
// The derivative acts on the measurement (so setpoint steps don't kick it) and is low-pass filtered,
// and the integral term is limited and stops growing while the output is saturated. Pass holdIntegral
// to stop it growing for other reasons too, such as the motor mixer running out of range.
class Pid
{
public:
    Pid(float kp, float ki, float kd, float outputLimit, float integralLimit, float derivativeCutoffHz);

    float update(float setpoint, float measurement, float dt, bool holdIntegral = false);
    void reset();

private:
    float kp, ki, kd;
    float outputLimit, integralLimit;
    float derivativeTimeConstant;    // of the derivative low-pass filter, in seconds
    float integral = 0;
    float filteredDerivative = 0;
    float lastMeasurement = 0;
    bool hasLastMeasurement = false;
};

#endif
