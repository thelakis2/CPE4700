#define _GNU_SOURCE
#include "encoder.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define ENCODER_MOTORS 2
#define COUNTS_PER_REV 700.0

static int position[ENCODER_MOTORS];
static int previous_a[ENCODER_MOTORS] = {-1, -1};
static double speed[ENCODER_MOTORS];
static struct timespec last_edge[ENCODER_MOTORS];
static char gpio_a[ENCODER_MOTORS][128];
static char gpio_b[ENCODER_MOTORS][128];
static int initialized;

static int read_gpio(const char *path)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0)
        return -errno;
    char value;
    ssize_t count = read(fd, &value, 1);
    close(fd);
    return count == 1 ? value == '1' : -EIO;
}

int enc_init(void)
{
    for (int motor = 0; motor < ENCODER_MOTORS; motor++) {
        const char *name = motor == 0 ? getenv("ENCODER_A0") : getenv("ENCODER_A1");
        const char *other = motor == 0 ? getenv("ENCODER_B0") : getenv("ENCODER_B1");
        if (!name)
            name = motor == 0 ? "/sys/class/gpio/gpio0/value" : "/sys/class/gpio/gpio1/value";
        if (!other)
            other = motor == 0 ? "/sys/class/gpio/gpio2/value" : "/sys/class/gpio/gpio3/value";
        snprintf(gpio_a[motor], sizeof(gpio_a[motor]), "%s", name);
        snprintf(gpio_b[motor], sizeof(gpio_b[motor]), "%s", other);
        position[motor] = 0;
        speed[motor] = 0.0;
        previous_a[motor] = read_gpio(gpio_a[motor]);
        clock_gettime(CLOCK_MONOTONIC, &last_edge[motor]);
    }
    initialized = 1;
    return 0;
}

double enc_get_speed(int motor)
{
    if (!initialized || motor < 0 || motor >= ENCODER_MOTORS)
        return 0.0;
    int current_a = read_gpio(gpio_a[motor]);
    int current_b = read_gpio(gpio_b[motor]);
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (current_a >= 0 && previous_a[motor] >= 0 && current_a != previous_a[motor]) {
        double elapsed = (now.tv_sec - last_edge[motor].tv_sec) +
                         (now.tv_nsec - last_edge[motor].tv_nsec) / 1e9;
        if (elapsed > 0.0) {
            int direction = current_a == current_b ? 1 : -1;
            speed[motor] = direction * (1.0 / elapsed) * 60.0 / COUNTS_PER_REV;
            position[motor] += direction;
        }
        last_edge[motor] = now;
    } else if (now.tv_sec - last_edge[motor].tv_sec > 0 ||
               now.tv_nsec - last_edge[motor].tv_nsec > 100000000) {
        speed[motor] = 0.0;
    }
    previous_a[motor] = current_a;
    return speed[motor];
}

int enc_get_position(int motor)
{
    return motor >= 0 && motor < ENCODER_MOTORS ? position[motor] : 0;
}

void enc_cleanup(void)
{
    initialized = 0;
}