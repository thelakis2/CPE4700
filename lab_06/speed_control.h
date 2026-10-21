#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H
void speed_control_init(void);
void speed_control(int motor, double target_speed);
void speed_control_cleanup(void);
int speed_control_set_encoder(int motor, int encoder_id, int encoder_sign);
int speed_control_set_gains(int motor, double kp, double ki);
/* Compute only: never writes PWM. dt and measured are supplied by caller. */
double speed_control_compute(int motor, double target, double measured, double dt, double limit);
void speed_control_reset(int motor);
#endif
