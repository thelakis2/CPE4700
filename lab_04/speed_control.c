#define _POSIX_C_SOURCE 200809L
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include "pwm.h"
#include "encoder.h"
#include "speed_control.h"

/* Starting values only: these are NOT experimentally tuned gains.
 * Confirm encoder scaling/sign and tune with wheels lifted.
 * May be overridden with -DSPEED_CONTROL_KP=... / -DSPEED_CONTROL_KI=... */
#ifndef SPEED_CONTROL_KP
#define SPEED_CONTROL_KP 0.002
#endif
#ifndef SPEED_CONTROL_KI
#define SPEED_CONTROL_KI 0.001
#endif

/* Matches the saturation in the supplied pwm.c. */
#define DUTY_LIMIT 0.80
#define FIRST_DT 0.001
#define MAX_DT 0.020

typedef struct {
    int encoder_id;
    int encoder_sign;
    double kp;
    double ki;
    double integral; /* Integral contribution in duty units. */
    struct timespec previous;
    bool have_previous;
} controller_t;

static controller_t controllers[2];
static bool initialized;

static bool valid_motor(int motor)
{
    return motor == 0 || motor == 1;
}

static double clamp(double value, double low, double high)
{
    return value < low ? low : (value > high ? high : value);
}

static void reset_state(controller_t *controller)
{
    controller->integral = 0.0;
    controller->have_previous = false;
}

void speed_control_init(void)
{
    /* Does not acquire/reinitialize GPIO or start encoder threads. */
    for (int motor = 0; motor < 2; ++motor) {
        controllers[motor] = (controller_t){
            .encoder_id = motor,
            .encoder_sign = 1,
            .kp = SPEED_CONTROL_KP,
            .ki = SPEED_CONTROL_KI
        };
        pwm_set_duty(motor, 0.0f);
    }
    initialized = true;
}

int speed_control_set_encoder(int motor, int encoder_id, int encoder_sign)
{
    if (!initialized || !valid_motor(motor) || !valid_motor(encoder_id) ||
        (encoder_sign != 1 && encoder_sign != -1))
        return -1;
    controllers[motor].encoder_id = encoder_id;
    controllers[motor].encoder_sign = encoder_sign;
    reset_state(&controllers[motor]);
    return 0;
}

int speed_control_set_gains(int motor, double kp, double ki)
{
    if (!initialized || !valid_motor(motor) || !isfinite(kp) ||
        !isfinite(ki) || kp < 0.0 || ki < 0.0)
        return -1;
    controllers[motor].kp = kp;
    controllers[motor].ki = ki;
    reset_state(&controllers[motor]);
    return 0;
}

void speed_control(int motor, double target_speed)
{
    if (!valid_motor(motor)) return;
    if (!initialized) {
        /* Lifecycle error: caller must initialize before controlling. */
        return;
    }
    controller_t *controller = &controllers[motor];
    struct timespec now;
    double measured = controller->encoder_sign *
                      encoder_get_speed(controller->encoder_id);

    if (!isfinite(target_speed) || !isfinite(measured) ||
        clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        pwm_set_duty(motor, 0.0f);
        reset_state(controller);
        return;
    }

    double dt = FIRST_DT;
    if (controller->have_previous) {
        dt = (double)(now.tv_sec - controller->previous.tv_sec)
           + (double)(now.tv_nsec - controller->previous.tv_nsec) / 1e9;
        /* Do not integrate across an unexpectedly long scheduling gap. */
        if (dt <= 0.0 || dt > MAX_DT) dt = 0.0;
    }
    controller->previous = now;
    controller->have_previous = true;

    double error = target_speed - measured;
    double proportional = controller->kp * error;
    double increment = controller->ki * error * dt;
    double candidate = clamp(controller->integral + increment,
                             -DUTY_LIMIT, DUTY_LIMIT);
    double candidate_output = proportional + candidate;

    /* Conditional integration: freeze if integration drives saturation
     * farther outward; permit integration that brings output back inward. */
    if ((candidate_output <= DUTY_LIMIT && candidate_output >= -DUTY_LIMIT) ||
        (candidate_output > DUTY_LIMIT && increment < 0.0) ||
        (candidate_output < -DUTY_LIMIT && increment > 0.0))
        controller->integral = candidate;

    double output = clamp(proportional + controller->integral,
                          -DUTY_LIMIT, DUTY_LIMIT);
    pwm_set_duty(motor, (float)output);
}

void speed_control_cleanup(void)
{
    if (!initialized) return;
    /* Must run before pwm_cleanup(), while PWM resources are available. */
    for (int motor = 0; motor < 2; ++motor) {
        pwm_set_duty(motor, 0.0f);
        reset_state(&controllers[motor]);
    }
    initialized = false;
}
