#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
 
#include "encoder.h"
#include "imu.h"
#include "pwm.h"
#include "bno055.h"
 
static void print_status(void)
{
    printf("enc0=%d speed0=%.4f | enc1=%d speed1=%.4f | heading=%.2f roll=%.2f pitch=%.2f\n",
           enc_get_count(0),
           enc_get_speed(0),
           enc_get_count(1),
           enc_get_speed(1),
           get_imu_heading(),
           get_imu_roll(),
           get_imu_pitch());
}
 
static int run_motor_demo(void)
{
    int channel;
 
    for (channel = 0; channel < PWM_CHANNEL_COUNT; ++channel) {
        printf("Testing channel %d\n", channel);
 
        if (pwm_set_duty(channel, 0.50f) != 0) {
            fprintf(stderr, "Failed to set forward duty on channel %d\n", channel);
            return -1;
        }
        sleep(1);
 
        if (pwm_set_duty(channel, -0.50f) != 0) {
            fprintf(stderr, "Failed to set reverse duty on channel %d\n", channel);
            return -1;
        }
        sleep(1);
 
        if (pwm_set_duty(channel, 0.0f) != 0) {
            fprintf(stderr, "Failed to stop channel %d\n", channel);
            return -1;
        }
    }
 
    return 0;
}
 
int main(int argc, char **argv)
{
    int i;
    int demo_mode = (argc > 1 && strcmp(argv[1], "--demo") == 0);
 
    if (enc_init() != 0) {
        fprintf(stderr, "Failed to initialize encoders\n");
        return 1;
    }
 
    if (imu_init() != 0) {
        fprintf(stderr, "Failed to initialize IMU\n");
        enc_cleanup();
        return 1;
    }
 
    if (pwm_init() != 0) {
        fprintf(stderr, "Failed to initialize PWM outputs\n");
        imu_cleanup();
        enc_cleanup();
        return 1;
    }
 
    if (demo_mode) {
        printf("Running motor demo mode\n");
        if (run_motor_demo() != 0) {
            pwm_cleanup();
            imu_cleanup();
            enc_cleanup();
            return 1;
        }
    }
    else {
        for (i = 0; i < 10; ++i) {
            print_status();
            sleep(1);
        }
    }
 
    pwm_cleanup();
    imu_cleanup();
    enc_cleanup();
    return 0;
}