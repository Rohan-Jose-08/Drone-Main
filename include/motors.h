#ifndef MOTORS_H
#define MOTORS_H

#include "pico/stdlib.h"
#include "mixer.h"

// ESC outputs as standard PWM: one pulse per motor every 1 / rateHz seconds, its width in microseconds.
void motors_init(const uint pins[MOTOR_COUNT], uint rateHz, float initialUs);
void motors_write(const float pulseUs[MOTOR_COUNT]);

#endif
