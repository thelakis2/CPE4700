#define _GNU_SOURCE
#include "../modules/encoder.h"
#include "../modules/pwm.h"
#include "../modules/speed_control.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t running = 1;

static void stop_handler(int signal_number)
{
    (void)signal_number;
    running = 0;
}

static void sleep_1ms(void)
{
    struct timespec interval = {0, 1000000};
    clock_nanosleep(CLOCK_MONOTONIC, 0, &interval, NULL);
}

static int step_response(int motor)
{
    FILE *log = fopen("step_response.csv", "w");
    if (!log)
        return -1;
    fprintf(log, "time_ms,duty_cycle,speed,position\n");
    const double duties[] = {0.0, 0.60, 0.0, -0.60, 0.60, 0.0};
    const int durations[] = {1000, 1000, 2000, 1000, 1000, 0};
    int elapsed_ms = 0;
    for (size_t segment = 0; segment < sizeof(duties) / sizeof(duties[0]) && running; segment++) {
        if (pwm_set_duty(motor, duties[segment]) != 0) {
            fclose(log);
            return -1;
        }
        for (int elapsed = 0; elapsed < durations[segment] && running; elapsed++) {
            double speed = enc_get_speed(motor);
            fprintf(log, "%d,%.6f,%.6f,%d\n", elapsed_ms, duties[segment], speed,
                    enc_get_position(motor));
            elapsed_ms++;
            sleep_1ms();
        }
    }
    if (pwm_set_duty(motor, 0.0) != 0) {
        fclose(log);
        return -1;
    }
    fclose(log);
    return 0;
}

static int speed_response(int motor)
{
    FILE *log = fopen("speed_control.csv", "w");
    if (!log)
        return -1;
    fprintf(log, "time_ms,target_speed,measured_speed\n");
    if (speed_control_init() != 0) {
        fclose(log);
        return -1;
    }

    const int cycle_duration_ms = 2000;
    const int cycle_count = 3;
    const int total_ms = cycle_duration_ms * cycle_count;
    int result = 0;
    for (int time_ms = 0; time_ms < total_ms && running; time_ms++) {
        double cycle_time = (time_ms % cycle_duration_ms) / 1000.0;
        double target_speed;
        if (cycle_time < 1.0) {
            target_speed = 100.0 - 200.0 * cycle_time;
        }
        else {
            target_speed = -100.0 + 200.0 * (cycle_time - 1.0);
        }  
        double measured_speed = enc_get_speed(motor);
        if (speed_control(motor, target_speed) != 0) {
            result = -1;
            break;
        }
        fprintf(log, "%d,%.6f,%.6f\n", time_ms, target_speed, measured_speed);
        sleep_1ms();
    }

    if (speed_control(motor, 0.0) != 0)
        result = -1;
    speed_control_cleanup();
    fclose(log);
    return result;
}

int main(int argc, char **argv)
{
    signal(SIGINT, stop_handler);
    if (pwm_init() != 0 || enc_init() != 0) {
        fprintf(stderr, "failed to initialize PWM or encoder\n");
        pwm_cleanup();
        enc_cleanup();
        return EXIT_FAILURE;
    }

    int result = 0;
    const char *mode = argc > 1 ? argv[1] : "step";
    if (strcmp(mode, "step") == 0)
        result = step_response(0);
    else if (strcmp(mode, "speed") == 0)
        result = speed_response(0);
    else if (strcmp(mode, "both") == 0) {
        result = step_response(0);
        if (result == 0 && running)
            result = speed_response(0);
    } else {
        fprintf(stderr, "usage: %s [step|speed|both]\n", argv[0]);
        result = -1;
    }
    pwm_cleanup();
    enc_cleanup();
    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}