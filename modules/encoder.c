#define _GNU_SOURCE
#include "encoder.h"

#include <gpiod.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* Jetson Orin Nano 40-pin header, gpiochip0 line offsets */
#define ENCODER_CHIP "gpiochip0"
#define ENCODER_MOTORS 2
static const unsigned int PIN_A[ENCODER_MOTORS] = {124, 52};
static const unsigned int PIN_B[ENCODER_MOTORS] = {53, 51};

/* Original scale factor from the working code: speed = SPEED_SCALE / delta_ns */
#define SPEED_SCALE (68220200.0 / 4.0)
#define DEBOUNCE_NS 100000
#define SPEED_SAMPLES 2

typedef _Atomic(double) atomic_double;

extern atomic_bool running; /* defined in robotest.c */

static atomic_long encoder_count[ENCODER_MOTORS];
static atomic_double encoder_speed[ENCODER_MOTORS];

static pthread_t thread_id[ENCODER_MOTORS];
static bool thread_started[ENCODER_MOTORS];
static struct gpiod_chip *chip;
static struct gpiod_line *line_a[ENCODER_MOTORS];
static struct gpiod_line *line_b[ENCODER_MOTORS];

static void *encoder_worker(void *arg)
{
    int id = (int)(long)arg;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(id + 4, &cpuset); /* cores 4 and 5 */
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (rc != 0)
        fprintf(stderr, "Encoder %d: RT priority failed: %s\n", id, strerror(rc));

    struct gpiod_line_event event;
    struct timespec timeout = {0, 100000000}; /* 100 ms */
    int64_t last_ns = 0;
    double speeds[SPEED_SAMPLES] = {0};
    int idx = 0;

    while (atomic_load(&running)) {
        int status = gpiod_line_event_wait(line_a[id], &timeout);

        if (status > 0) {
            if (gpiod_line_event_read(line_a[id], &event) < 0)
                continue;

            int64_t now_ns = (int64_t)event.ts.tv_sec * 1000000000LL + event.ts.tv_nsec;
            if (last_ns == 0) {
                last_ns = now_ns;
                continue;
            }
            int64_t delta_ns = now_ns - last_ns;
            if (delta_ns <= DEBOUNCE_NS)
                continue;
            last_ns = now_ns;

            int state_a = (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) ? 1 : 0;
            int state_b = gpiod_line_get_value(line_b[id]);
            bool forward = (state_a == state_b);

            if (forward) {
                atomic_fetch_add(&encoder_count[id], 1);
            } else {
                atomic_fetch_sub(&encoder_count[id], 1);
                delta_ns = -delta_ns;
            }

            speeds[idx++ % SPEED_SAMPLES] = SPEED_SCALE / (double)delta_ns;
            double sum = 0.0;
            for (int i = 0; i < SPEED_SAMPLES; i++)
                sum += speeds[i];
            atomic_store(&encoder_speed[id], sum / SPEED_SAMPLES);
        } else if (status == 0) {
            /* No edges for 100 ms: motor is stopped */
            for (int i = 0; i < SPEED_SAMPLES; i++)
                speeds[i] = 0.0;
            atomic_store(&encoder_speed[id], 0.0);
            last_ns = 0;
        } else {
            fprintf(stderr, "Encoder %d: event wait failed\n", id);
            break;
        }
    }
    return NULL;
}

int enc_init(void)
{
    chip = gpiod_chip_open_by_name(ENCODER_CHIP);
    if (!chip) {
        fprintf(stderr, "[ERROR] Failed to open chip %s\n", ENCODER_CHIP);
        return -1;
    }

    for (int id = 0; id < ENCODER_MOTORS; id++) {
        atomic_store(&encoder_count[id], 0);
        atomic_store(&encoder_speed[id], 0.0);

        line_a[id] = gpiod_chip_get_line(chip, PIN_A[id]);
        line_b[id] = gpiod_chip_get_line(chip, PIN_B[id]);
        if (!line_a[id] || !line_b[id]) {
            fprintf(stderr, "[ERROR] Encoder %d: failed to get lines\n", id);
            return -1;
        }
        if (gpiod_line_request_both_edges_events(line_a[id],
                id == 0 ? "Encoder_A0" : "Encoder_A1") < 0) {
            fprintf(stderr, "[ERROR] Encoder %d: edge request on A failed\n", id);
            return -1;
        }
        if (gpiod_line_request_input(line_b[id], id == 0 ? "Encoder_B0" : "Encoder_B1") < 0) {
            fprintf(stderr, "[ERROR] Encoder %d: input request on B failed\n", id);
            return -1;
        }
    }

    for (int id = 0; id < ENCODER_MOTORS; id++) {
        if (pthread_create(&thread_id[id], NULL, encoder_worker, (void *)(long)id) != 0) {
            fprintf(stderr, "[ERROR] Failed to create encoder thread %d\n", id);
            return -1;
        }
        thread_started[id] = true;
    }
    return 0;
}

double enc_get_speed(int motor)
{
    if (motor < 0 || motor >= ENCODER_MOTORS)
        return 0.0;
    return atomic_load(&encoder_speed[motor]);
}

int enc_get_position(int motor)
{
    if (motor < 0 || motor >= ENCODER_MOTORS)
        return 0;
    return (int)atomic_load(&encoder_count[motor]);
}

void enc_cleanup(void)
{
    /* The caller clears `running` first; threads then exit and are joined. */
    for (int id = 0; id < ENCODER_MOTORS; id++) {
        if (thread_started[id]) {
            pthread_join(thread_id[id], NULL);
            thread_started[id] = false;
        }
        if (line_a[id]) {
            gpiod_line_release(line_a[id]);
            line_a[id] = NULL;
        }
        if (line_b[id]) {
            gpiod_line_release(line_b[id]);
            line_b[id] = NULL;
        }
    }
    if (chip) {
        gpiod_chip_close(chip);
        chip = NULL;
    }
}