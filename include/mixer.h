#ifndef MIXER_H
#define MIXER_H

#define MOTOR_COUNT 4

// Quad X, seen from above with the front at the top. Diagonal motors spin the same way:
//
//   M4 front left, clockwise          M2 front right, anticlockwise
//   M3 rear left, anticlockwise       M1 rear right, clockwise
//
// Sign conventions: positive roll lowers the right side, positive pitch raises the nose,
// and positive yaw turns the drone clockwise seen from above.

// Turns a throttle pulse and roll/pitch/yaw corrections (all in microseconds) into one pulse per motor,
// each within minUs..maxUs. The throttle is shifted as needed so the full correction fits, which keeps
// attitude control ahead of throttle. If the corrections need more range than there is, they are also
// scaled down together, keeping their ratio, and this returns true (saturated).
bool mix(float throttleUs, float roll, float pitch, float yaw, float minUs, float maxUs, float motorUs[MOTOR_COUNT]);

#endif
