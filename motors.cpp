#include <math.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "include/motors.h"

static uint motorPins[MOTOR_COUNT];

void motors_init(const uint pins[MOTOR_COUNT], uint rateHz, float initialUs)
{
    // Run each PWM counter at 1 MHz so one count is one microsecond.
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&config, 1000000 / rateHz - 1);

    // Two pins can share a PWM slice, and pwm_init clears both of its channels,
    // so set up every slice before setting any pulse widths.
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motorPins[i] = pins[i];
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
        pwm_init(pwm_gpio_to_slice_num(pins[i]), &config, false);
    }

    float initial[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        initial[i] = initialUs;
    }
    motors_write(initial);

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        pwm_set_enabled(pwm_gpio_to_slice_num(pins[i]), true);
    }
}

void motors_write(const float pulseUs[MOTOR_COUNT])
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        pwm_set_gpio_level(motorPins[i], (uint16_t)lroundf(pulseUs[i]));
    }
}
