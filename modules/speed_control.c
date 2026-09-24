#include "speed_control.h"
#include "encoder.h"
#include "pwm.h"

#define SPEED_KP 0.004

int speed_control_init(void)
{
    return 0;
}

int speed_control(int motor, double target_speed)
{
    double error = target_speed - enc_get_speed(motor);
    double command = SPEED_KP * error;
    return pwm_set_duty(motor, command);
}

void speed_control_cleanup(void)
{
    pwm_set_duty(0, 0.0);
    pwm_set_duty(1, 0.0);
}