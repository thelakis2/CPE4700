#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "imu.h"
#include "encoder.h"
#include "pwm.h"
#include "balance_loop.h"
#include "gui.h"

#define LOG_CAPACITY      600000
#define CONTROL_PERIOD_NS 1000000L
#define CONTROL_PRIORITY  80
#define LOG_FILENAME      "balance_log.csv"

/* Shared with the encoder threads and GUI. */
atomic_bool running = ATOMIC_VAR_INIT(true);

/* Set by the signal handler; checked by the balance worker. */
static volatile sig_atomic_t interrupted;


/* --------------------------------------------------------------------------
 * Logging
 * -------------------------------------------------------------------------- */

typedef struct {
    double time_ms;
    double pitch_deg;

    double motor0_rpm;
    double motor1_rpm;

    double duty0;
    double duty1;

    double kp;
    double ki;
    double kd;

    int enabled;
    int fault;
} log_row_t;

static log_row_t log_rows[LOG_CAPACITY];
static size_t log_count;


/* --------------------------------------------------------------------------
 * Signal and timing helpers
 * -------------------------------------------------------------------------- */

static void handle_signal(int signal_number)
{
    (void)signal_number;
    interrupted = 1;
}

static double elapsed_seconds(struct timespec start, struct timespec end)
{
    return (double)(end.tv_sec - start.tv_sec)
         + (double)(end.tv_nsec - start.tv_nsec) / 1e9;
}

static void advance_deadline(struct timespec *deadline)
{
    deadline->tv_nsec += CONTROL_PERIOD_NS;

    if (deadline->tv_nsec >= 1000000000L) {
        deadline->tv_nsec -= 1000000000L;
        ++deadline->tv_sec;
    }
}


/* --------------------------------------------------------------------------
 * Balance-control worker
 * -------------------------------------------------------------------------- */

static void *balance_worker(void *argument)
{
    (void)argument;

    /* Request FIFO scheduling for this worker thread. */
    struct sched_param priority = {
        .sched_priority = CONTROL_PRIORITY
    };

    int status = pthread_setschedparam(
        pthread_self(),
        SCHED_FIFO,
        &priority
    );

    if (status != 0) {
        fprintf(
            stderr,
            "FIFO scheduling unavailable: %s\n",
            strerror(status)
        );
    }

    struct timespec start_time;
    struct timespec next_deadline;
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &start_time) != 0) {
        atomic_store(&running, false);
        return NULL;
    }

    next_deadline = start_time;

    for (unsigned int iteration = 0;
         atomic_load(&running) &&
         !interrupted &&
         log_count < LOG_CAPACITY;
         ++iteration) {

        /* Run one balance-control update. */
        balance_loop_step(iteration);

        if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
            break;
        }

        /* Capture telemetry in RAM rather than writing during control. */
        log_row_t *row = &log_rows[log_count++];

        row->time_ms = 1000.0 * elapsed_seconds(start_time, now);
        row->pitch_deg = atomic_load(&g_pitch_deg);

        row->motor0_rpm = atomic_load(&g_motor_rpm[0]);
        row->motor1_rpm = atomic_load(&g_motor_rpm[1]);

        row->duty0 = atomic_load(&g_duty[0]);
        row->duty1 = atomic_load(&g_duty[1]);

        balance_loop_get_gains(&row->kp, &row->ki, &row->kd);

        row->enabled = atomic_load(&g_enabled);
        row->fault = atomic_load(&g_fault);

        /* Skip expired deadlines instead of performing burst updates. */
        advance_deadline(&next_deadline);

        while (elapsed_seconds(now, next_deadline) <= 0.0) {
            advance_deadline(&next_deadline);
        }

        /* Sleep until the next absolute deadline. */
        do {
            status = clock_nanosleep(
                CLOCK_MONOTONIC,
                TIMER_ABSTIME,
                &next_deadline,
                NULL
            );
        } while (status == EINTR && !interrupted);

        if (status != 0 && status != EINTR) {
            break;
        }
    }

    balance_loop_cleanup();
    atomic_store(&running, false);

    return NULL;
}


/* --------------------------------------------------------------------------
 * Application entry point
 * -------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    /* Install Ctrl+C and termination handlers. */
    struct sigaction signal_action = {0};

    signal_action.sa_handler = handle_signal;
    sigemptyset(&signal_action.sa_mask);

    if (sigaction(SIGINT, &signal_action, NULL) != 0 ||
        sigaction(SIGTERM, &signal_action, NULL) != 0) {
        perror("sigaction");
        return EXIT_FAILURE;
    }

    /* Verify that the log can be opened before initializing hardware. */
    FILE *log_file = fopen(LOG_FILENAME, "w");

    if (log_file == NULL) {
        perror(LOG_FILENAME);
        return EXIT_FAILURE;
    }

    bool pwm_initialized = false;
    bool imu_initialized = false;
    bool encoder_initialized = false;
    bool worker_started = false;

    pthread_t worker_thread;
    int exit_status = EXIT_FAILURE;

    /* Initialize PWM and explicitly stop both motors. */
    if (pwm_init() != 0) {
        goto cleanup;
    }

    pwm_initialized = true;

    pwm_set_duty(0, 0.0f);
    pwm_set_duty(1, 0.0f);

    /* Initialize the IMU. */
    if (imu_init() != 0) {
        goto cleanup;
    }

    imu_initialized = true;

    /* Initialize the encoders and balance controller. */
    if (encoder_init() != 0) {
        goto cleanup;
    }

    encoder_initialized = true;

    balance_loop_init();

    /* Start hardware control in a separate thread. */
    int status = pthread_create(
        &worker_thread,
        NULL,
        balance_worker,
        NULL
    );

    if (status != 0) {
        fprintf(stderr, "pthread_create: %s\n", strerror(status));
        goto cleanup;
    }

    worker_started = true;

    /* GTK runs on the main thread until the GUI exits. */
    gui_init(argc, argv);

    exit_status = EXIT_SUCCESS;


cleanup:
    /* Stop the worker before releasing hardware resources. */
    atomic_store(&running, false);

    if (worker_started) {
        pthread_join(worker_thread, NULL);
    }

    if (pwm_initialized) {
        balance_loop_cleanup();
    }

    if (encoder_initialized) {
        encoder_cleanup();
    }

    if (imu_initialized) {
        imu_cleanup();
    }

    if (pwm_initialized) {
        pwm_cleanup();
    }

    /* Write buffered measurements after hardware shutdown. */
    const char *csv_header =
        "time_ms,pitch_deg,motor0_rpm,motor1_rpm,"
        "duty0,duty1,kp,ki,kd,enabled,fault\n";

    if (fputs(csv_header, log_file) == EOF) {
        exit_status = EXIT_FAILURE;
    }

    for (size_t index = 0; index < log_count; ++index) {
        const log_row_t *row = &log_rows[index];

        int write_status = fprintf(
            log_file,
            "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n",
            row->time_ms,
            row->pitch_deg,
            row->motor0_rpm,
            row->motor1_rpm,
            row->duty0,
            row->duty1,
            row->kp,
            row->ki,
            row->kd,
            row->enabled,
            row->fault
        );

        if (write_status < 0) {
            exit_status = EXIT_FAILURE;
            break;
        }
    }

    if (fclose(log_file) != 0) {
        exit_status = EXIT_FAILURE;
    }

    printf(
        "Saved %zu samples to %s\n",
        log_count,
        LOG_FILENAME
    );

    return exit_status;
}