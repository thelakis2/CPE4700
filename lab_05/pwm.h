#ifndef PWM_H
#define PWM_H

// Initialize the pwm module
int pwm_init();

// Setter method for PWM duty cycle (0.0 to 1.0)
int pwm_set_duty(int channel, float duty_cycle);

void pwm_cleanup(void);

#endif // PWM_H
