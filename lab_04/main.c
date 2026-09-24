#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>
#include <stdatomic.h>

#include "encoder.h"
#include "pwm.h"
#include "speed_control.h"
#include "step_test.h"

#define MOTOR 0
#define LOOP_PERIOD_NS 1000000L   
#define CYCLE_PERIOD_S 2.0     
#define NUM_CYCLES 3

atomic_bool running = true;

static void handle_sigint(int sig)
{
    (void)sig;
    atomic_store(&running, false);
}

static double triangle_reference(double cycle_time)
{
    if (cycle_time < 1.0)
    {
        return 100.0 - 200.0 * cycle_time;
    }
    else
    {
        return -100.0 + 200.0 * (cycle_time - 1.0);
    }
}

int main(void)
{
    signal(SIGINT, handle_sigint);

    if (pwm_init() != 0)
    {
        fprintf(stderr, "[ERROR] pwm_init failed\n");
        return EXIT_FAILURE;
    }

    if (encoder_init() != 0)
    {
        fprintf(stderr, "[ERROR] encoder_init failed\n");
        pwm_cleanup();
        return EXIT_FAILURE;
    }

    struct timespec ts = {0, LOOP_PERIOD_NS};

    step_test_init();

    unsigned int loop_count = 0;
    while (atomic_load(&running))
    {
        int still_running = step_test(loop_count);
        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
        loop_count++;
        if (still_running == 0)
        {
            break;
        }
    }

    step_test_cleanup();

    if (speed_control_init() != 0)
    {
        fprintf(stderr, "[ERROR] speed_control_init failed\n");
        encoder_cleanup();
        pwm_cleanup();
        return EXIT_FAILURE;
    }

    FILE *speed_log = fopen("speed_control.csv", "w");
    if (speed_log == NULL)
    {
        perror("speed_control.csv");
        speed_control_cleanup();
        encoder_cleanup();
        pwm_cleanup();
        return EXIT_FAILURE;
    }
    fprintf(speed_log, "time_ms,target_speed,measured_speed\n");

    long t_ms = 0;
    int cycle_num = 0;

    while (atomic_load(&running) && cycle_num < NUM_CYCLES)
    {
        double elapsed_s = (double)t_ms / 1000.0;
        double cycle_time = elapsed_s - cycle_num * CYCLE_PERIOD_S;

        if (cycle_time >= CYCLE_PERIOD_S)
        {
            cycle_num++;
            if (cycle_num >= NUM_CYCLES)
            {
                break;
            }
            cycle_time -= CYCLE_PERIOD_S;
        }

        double target_speed = triangle_reference(cycle_time);
        double measured_speed = encoder_get_speed(MOTOR);

        speed_control(MOTOR, target_speed);

        fprintf(speed_log, "%ld,%f,%f\n", t_ms, target_speed, measured_speed);

        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
        t_ms += 1;
    }

    for (int i = 0; i < 200; i++)
    {
        speed_control(MOTOR, 0.0);
        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
    }

    fflush(speed_log);
    fclose(speed_log);

    speed_control_cleanup();
    encoder_cleanup();
    pwm_cleanup();

    printf("[DONE] step_response.csv and speed_control.csv written. "
           "Completed %d triangular cycles.\n", cycle_num);
    return EXIT_SUCCESS;
}