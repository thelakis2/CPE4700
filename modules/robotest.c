#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <math.h>
#include <signal.h>

//sudo apt update
//sudo apt install libgpiod2 libgpiod-dev

// gcc -o robotest robotest.c encoder.c pwm.c speed_loop.c imu.c bno055.c -lgpiod -lpthread -lm

#include "encoder.h"
#include "pwm.h"
#include "imu.h"
#include "step_test.h"

#define INTERVAL_NS 1000000 // 1 millisecond in nanoseconds
#define PRIORITY 80         // Real-time priority (1-99)clear
#define MOTOR 0
#define CYCLE_MS 2000
#define NUM_CYCLES 3

    // Define the atomic boolean
atomic_bool running = true;

void handle_sigint(int sig)
{
    (void)sig;
    atomic_store(&running, false);
}

//int main(int argc, char *argv[])
int main(void)
{
    signal(SIGINT, handle_sigint);

    struct sched_param param;
    struct timespec next_time;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) { perror("mlockall failed"); exit(EXIT_FAILURE); }

    memset(&param, 0, sizeof(param));
    param.sched_priority = PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) { perror("sched_setscheduler failed (run as root)"); exit(EXIT_FAILURE); }

    if (clock_gettime(CLOCK_MONOTONIC, &next_time) == -1) { perror("clock_gettime failed"); exit(EXIT_FAILURE); }

    if (imu_init()) { fprintf(stderr, "Failed to initialize IMU\n"); exit(EXIT_FAILURE); }
    if (enc_init()) { fprintf(stderr, "Failed to initialize encoders\n"); exit(EXIT_FAILURE); }
    if (pwm_init()) { fprintf(stderr, "Failed to initialize pwms\n"); exit(EXIT_FAILURE); }

    step_test_init();

    run_phase_t run_phase = RUN_STEP_TEST;
    FILE *speed_log = NULL;
    unsigned int speed_start = 0;

    printf("Real-time loop started at 1ms intervals.\n");

    for (unsigned int loop_count = 0; running && loop_count < 1000000; ++loop_count)
    {
        next_time.tv_nsec += INTERVAL_NS;
        if (next_time.tv_nsec >= 1000000000) { next_time.tv_sec += 1; next_time.tv_nsec -= 1000000000; }

        if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL) != 0)
        {
            perror("clock_nanosleep failed");
            break;
        }

        if (run_phase == RUN_STEP_TEST)
        {
            if (step_test(loop_count) == 0)   // 0 = step test finished
            {
                speed_control_init();
                speed_log = fopen("speed_control.csv", "w");
                if (!speed_log) { perror("speed_control.csv"); break; }
                fprintf(speed_log, "time_ms,target_speed,measured_speed\n");
                speed_start = loop_count + 1;
                run_phase = RUN_SPEED_CONTROL;
            }
        }
        else
        {
            unsigned int t_ms = loop_count - speed_start;
            if (t_ms >= NUM_CYCLES * CYCLE_MS)
            {
                speed_control(MOTOR, 0.0);    // command zero, then finish
                break;
            }

            double cycle_time = (t_ms % CYCLE_MS) / 1000.0;
            double target_speed = (cycle_time < 1.0)
                ? 100.0 - 200.0 * cycle_time
                : -100.0 + 200.0 * (cycle_time - 1.0);

            double measured_speed = enc_get_speed(MOTOR);
            speed_control(MOTOR, target_speed);
            fprintf(speed_log, "%u,%f,%f\n", t_ms, target_speed, measured_speed);
        }
    }

    if (speed_log) fclose(speed_log);
    atomic_store(&running, false);

    speed_control_cleanup();   // zeros both PWM outputs, so call before pwm_cleanup
    step_test_cleanup();
    enc_cleanup();
    pwm_cleanup();

    munlockall();
    return 0;
}


    atomic_store(&running, false); // Signal other threads to stop

    enc_cleanup();
    pwm_cleanup();
    step_test_cleanup();

    // Unlock memory before exiting
    munlockall();
    return 0;
}
