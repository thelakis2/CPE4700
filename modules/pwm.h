#ifndef PWM_H
#define PWM_H
/*oliukjawe*/
int pwm_init(void);
int pwm_set_duty(int motor, double duty);
void pwm_cleanup(void);

#endif