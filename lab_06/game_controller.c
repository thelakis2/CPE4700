#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "pwm.h"
#include "encoder.h"
#include "speed_control.h"

/* --------------------------------------------------------------------------
 * Configuration
 *
 * Verify these against the event monitor and physical wiring.
 * -------------------------------------------------------------------------- */

#ifndef LEFT_CHANNEL
#define LEFT_CHANNEL 0
#endif

#ifndef ENCODER_ID
#define ENCODER_ID 0
#endif

#ifndef ENCODER_SIGN
#define ENCODER_SIGN 1
#endif

#ifndef LEFT_SIGN
#define LEFT_SIGN 1
#endif

#ifndef RIGHT_SIGN
#define RIGHT_SIGN 1
#endif

#ifndef STEER_SIGN
#define STEER_SIGN 1
#endif

#ifndef SLOW_SPEED
#define SLOW_SPEED 50.0
#endif

#ifndef FAST_SPEED
#define FAST_SPEED 90.0
#endif

#define LIMIT       0.8
#define STEER       0.4
#define PERIOD_NS   1000000LL
#define LOG_NS      20000000LL
#define MAX_ROWS    30000

/* --------------------------------------------------------------------------
 * Types and global state
 * -------------------------------------------------------------------------- */

typedef struct {
    double ms;
    double target;
    double speed;

    float left;
    float right;

    int mode;
} row_t;

typedef struct {
    bool up;
    bool down;
    bool left;
    bool right;
    bool stop;
    bool fast;
    bool armed;
} input_t;

atomic_bool running = ATOMIC_VAR_INIT(true);

static volatile sig_atomic_t quit;

static row_t rows[MAX_ROWS];
static size_t count;

/* --------------------------------------------------------------------------
 * Utility functions
 * -------------------------------------------------------------------------- */

static void interrupted(int sig)
{
    (void)sig;
    quit = 1;
}

static long long clock_ns(void)
{
    struct timespec t;

    if (clock_gettime(CLOCK_MONOTONIC, &t)) {
        return -1;
    }

    return (long long)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void zero(void)
{
    pwm_set_duty(0, 0);
    pwm_set_duty(1, 0);
}

static double clip(double v)
{
    return fmax(-LIMIT, fmin(LIMIT, v));
}

/* --------------------------------------------------------------------------
 * Gamepad input handling
 * -------------------------------------------------------------------------- */

static void update(input_t *s, const struct input_event *e)
{
    if (e->type == EV_KEY) {
        bool on = e->value != 0;

        switch (e->code) {
        case BTN_DPAD_UP:
            s->up = on;
            break;

        case BTN_DPAD_DOWN:
            s->down = on;
            break;

        case BTN_DPAD_LEFT:
            s->left = on;
            break;

        case BTN_DPAD_RIGHT:
            s->right = on;
            break;

        case BTN_B:
            s->stop = on;

            if (on) {
                s->armed = false;
            }
            break;

        case BTN_X:
            if (e->value == 1) {
                s->fast = !s->fast;
            }
            break;

        case BTN_START:
            if (e->value == 1) {
                quit = 1;
            }
            break;

        default:
            break;
        }
    } else if (e->type == EV_ABS) {
        if (e->code == ABS_HAT0Y) {
            s->up = e->value < 0;
            s->down = e->value > 0;
        }

        if (e->code == ABS_HAT0X) {
            s->left = e->value < 0;
            s->right = e->value > 0;
        }
    }
}

/* --------------------------------------------------------------------------
 * Main application
 * -------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(
            stderr,
            "Usage: %s /dev/input/eventX output.csv\n",
            argv[0]
        );

        return 1;
    }

    int result = 1;
    int fd = -1;

    FILE *log = NULL;

    bool pwm = false;
    bool enc = false;
    bool controller = false;

    input_t in = {0};

    /* Set up signal handling. */

    struct sigaction sa = {0};

    sa.sa_handler = interrupted;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGINT, &sa, NULL) ||
        sigaction(SIGTERM, &sa, NULL)) {
        perror("sigaction");
        return 1;
    }

    /* Open the gamepad without blocking the control loop. */

    fd = open(argv[1], O_RDONLY | O_NONBLOCK | O_CLOEXEC);

    if (fd < 0) {
        perror("gamepad open");
        goto cleanup;
    }

    char name[256] = {0};

    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) {
        perror("input device name");
        goto cleanup;
    }

    printf(
        "Device: %s. B=stop, X=mode, START=exit. Starts disarmed.\n",
        name
    );

    /* Create a new log file. */

    log = fopen(argv[2], "wx");

    if (!log) {
        perror("CSV (use a new filename)");
        goto cleanup;
    }

    /* Initialize the hardware and speed controller. */

    if (pwm_init() != 0) {
        fprintf(stderr, "PWM initialization failed\n");
        goto cleanup;
    }

    pwm = true;
    zero();

    /*
     * encoder_cleanup supports joining partially started threads only after
     * successful init; fix the legacy encoder initializer before relying on
     * recovery from partial failure.
     */
    if (encoder_init() != 0) {
        fprintf(stderr, "Encoder initialization failed\n");
        goto cleanup;
    }

    enc = true;

    speed_control_init();
    controller = true;

    /* Initialize control and logging deadlines. */

    long long start = clock_ns();

    if (start < 0) {
        perror("clock");
        goto cleanup;
    }

    long long previous = start;
    long long deadline = start;
    long long next_log = start + LOG_NS;

    /* ----------------------------------------------------------------------
     * Control loop
     * ---------------------------------------------------------------------- */

    while (!quit) {
        /* Check for input-device errors or disconnection. */

        struct pollfd p = {
            .fd = fd,
            .events = POLLIN
        };

        int pr = poll(&p, 1, 0);

        if (pr < 0 && errno == EINTR) {
            continue;
        }

        if (pr < 0 || (p.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            fprintf(stderr, "Gamepad lost\n");
            goto cleanup;
        }

        /*
         * Bounded drain: fail closed rather than let an event flood
         * starve control.
         */
        int n;

        for (n = 0; n < 256; n++) {
            struct input_event e;
            ssize_t bytes = read(fd, &e, sizeof(e));

            if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                break;
            }

            if (bytes < 0 && errno == EINTR) {
                if (quit) {
                    break;
                }

                continue;
            }

            if (bytes != (ssize_t)sizeof(e)) {
                fprintf(stderr, "Input read failed\n");
                goto cleanup;
            }

            if (e.type == EV_SYN && e.code == SYN_DROPPED) {
                fprintf(stderr, "Input events lost\n");
                goto cleanup;
            }

            update(&in, &e);

            if (in.stop || quit) {
                zero();
                speed_control_reset(0);
            }

            if (quit) {
                break;
            }
        }

        if (n == 256) {
            fprintf(stderr, "Input flood\n");
            goto cleanup;
        }

        if (quit) {
            break;
        }

        /*
         * Rearm only after all D-pad directions are released
         * following stop/start.
         */
        if (!in.stop && !in.up && !in.down && !in.left && !in.right) {
            in.armed = true;
        }

        /* Check elapsed control-loop time. */

        long long now = clock_ns();

        if (now < 0) {
            perror("clock");
            goto cleanup;
        }

        double dt = (now - previous) / 1e9;
        previous = now;

        if (dt > 0.020) {
            fprintf(
                stderr,
                "Control deadline missed (>20 ms); stopping\n"
            );

            goto cleanup;
        }

        /* Read encoder feedback and determine the target speed. */

        double measured = ENCODER_SIGN * encoder_get_speed(ENCODER_ID);

        if (!isfinite(measured)) {
            fprintf(stderr, "Invalid encoder speed\n");
            goto cleanup;
        }

        int direction = (int)in.up - (int)in.down;
        bool enabled = in.armed && !in.stop && direction != 0;

        double target = enabled
            ? direction * (in.fast ? FAST_SPEED : SLOW_SPEED)
            : 0;

        /* Calculate final motor commands. */

        float left = 0;
        float right = 0;

        if (enabled) {
            double steer =
                STEER_SIGN * ((int)in.right - (int)in.left) * STEER;

            /*
             * Reserve steering headroom so final clamping
             * does not wind up the PI.
             */
            double base = speed_control_compute(
                0,
                target,
                measured,
                dt,
                LIMIT - fabs(steer)
            );

            left = (float)(LEFT_SIGN * clip(base - steer));
            right = (float)(RIGHT_SIGN * clip(base + steer));
        } else {
            speed_control_reset(0);
        }

        if (quit) {
            break;
        }

        pwm_set_duty(LEFT_CHANNEL, left);
        pwm_set_duty(1 - LEFT_CHANNEL, right);

        /* Buffer one log sample per scheduled logging interval. */

        if (now >= next_log) {
            if (now - next_log >= LOG_NS) {
                fprintf(stderr, "Missed 20 ms log slot; stopping\n");
                goto cleanup;
            }

            if (count == MAX_ROWS) {
                fprintf(stderr, "Ten-minute log buffer full\n");
                goto cleanup;
            }

            rows[count++] = (row_t) {
                (now - start) / 1e6,
                target,
                measured,
                left,
                right,
                in.fast
            };

            next_log += LOG_NS;
        }

        /* Wait until the next absolute control-loop deadline. */

        deadline += PERIOD_NS;

        long long end = clock_ns();

        if (end < 0) {
            goto cleanup;
        }

        while (deadline <= end) {
            deadline += PERIOD_NS;
        }

        struct timespec wake = {
            .tv_sec = deadline / 1000000000LL,
            .tv_nsec = deadline % 1000000000LL
        };

        int rc;

        do {
            rc = clock_nanosleep(
                CLOCK_MONOTONIC,
                TIMER_ABSTIME,
                &wake,
                NULL
            );
        } while (rc == EINTR && !quit);

        if (rc && rc != EINTR) {
            fprintf(stderr, "Sleep: %s\n", strerror(rc));
            goto cleanup;
        }
    }

    result = 0;

cleanup:
    /* Stop the motors before releasing hardware resources. */

    if (pwm) {
        zero();
    }

    if (controller) {
        speed_control_cleanup();
    }

    atomic_store(&running, false);

    if (enc) {
        encoder_cleanup();
    }

    if (pwm) {
        pwm_cleanup();
    }

    if (fd >= 0) {
        close(fd);
    }

    /* Write buffered samples after the motors have stopped. */

    if (log) {
        if (fputs(
                "time_ms,target_speed,measured_speed,"
                "left_pwm,right_pwm,mode\n",
                log
            ) == EOF) {
            result = 1;
        }

        for (size_t i = 0; i < count; i++) {
            row_t *r = &rows[i];

            if (fprintf(
                    log,
                    "%.3f,%.6f,%.6f,%.9g,%.9g,%d\n",
                    r->ms,
                    r->target,
                    r->speed,
                    r->left,
                    r->right,
                    r->mode
                ) < 0) {
                result = 1;
                break;
            }
        }

        if (fclose(log)) {
            perror("CSV close");
            result = 1;
        }
    }

    return result;
}