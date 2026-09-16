# include <stdio.h>
# include <unistd.h>
# include "pwm.h"

int main (void) {
    if (pwm_init() != 0) {
        printf (" PWM init failed \n ") ;
        return -1;
    }
    
    for (float dc = 0; dc <= 1.0; dc += 0.05) {
        pwm_set_duty (0, dc) ;
        pwm_set_duty (1, 1 - dc) ;
        usleep (50000) ;
    }

    pwm_cleanup();

    return 0;
}