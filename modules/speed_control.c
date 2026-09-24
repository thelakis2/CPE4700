#include "speed_control.h"
#include "encoder.h"
#include "pwm.h"

#define SPEED_KP 0.004

int speed_control_init(void)
{
    return 0;
}
/*l;kjes;lksw*/
int speed_control(int motor, double target_speed)
{
    double measured_speed = encoder_get_speed(motor);
    double error = target_speed - measured_speed;
    double command = SPEED_KP * error;

    if (command > 0.8)
    {
        command = 0.8;
    }
    else if (command < -0.8)
    {
        command = -0.8;
    }

    return pwm_set_duty(motor, (float)command);
}

void speed_control_cleanup(void)
{
    pwm_set_duty(0, 0.0f);
    pwm_set_duty(1, 0.0f);
}