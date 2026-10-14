# ifndef PWM_H
# define PWM_H

#define PWM_CHANNEL_COUNT 2

int pwm_init (void);
int pwm_set_duty (int channel, float duty_cycle) ;
void pwm_cleanup (void) ;

# endif