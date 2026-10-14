#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

/* PWM and encoder modules must already be initialized by the application.
 * Target speed uses the same units/sign as encoder_get_speed(). */
void speed_control_init(void);
void speed_control(int motor, double target_speed);
void speed_control_cleanup(void);

/* Configure while the motor is stopped, before running a test.
 * encoder_sign must be +1 or -1. Returns 0 on success, -1 on error.
 * Defaults: motor 0 -> encoder 0; motor 1 -> encoder 1; signs +1. */
int speed_control_set_encoder(int motor, int encoder_id, int encoder_sign);

/* Gains: duty/speed-unit and duty/(speed-unit * second).
 * Call after init, while stopped. Resets that motor's integrator.
 * Returns 0 on success, -1 on invalid arguments. */
int speed_control_set_gains(int motor, double kp, double ki);

#endif
