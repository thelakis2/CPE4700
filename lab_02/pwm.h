# ifndef PWM_H
# define PWM_H

int pwm_init (void);
int pwm_set_duty (int channel, float duty_cycle) ;
int pwm_cleanup (void) ;

# endif