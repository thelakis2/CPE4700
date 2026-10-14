#define _GNU_SOURCE
#include "pwm.h"

#include <errno.h>
#include <fcntl.h>
#include <gpiod.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ---------- PWM (sysfs) ----------
 * motor 0: pin 33 -> pwmchip2/pwm0
 * motor 1: pin 32 -> pwmchip3/pwm0
 */
#define PWM_MOTORS 2
#define PWM_CHANNEL "0"
#define PWM_PERIOD_NS 200000 /* 5 kHz (matches the known-good driver code) */
#define PWM_PERIOD_STR "200000"
#define PWM_LIMIT 0.8

static const char *chip_paths[PWM_MOTORS] = {
    "/sys/class/pwm/pwmchip2",
    "/sys/class/pwm/pwmchip3",
};

/* ---------- Direction GPIO (MDD10 DIR pins, libgpiod v1) ----------
 * motor 0: pin 31, PQ.06, line 106
 * motor 1: pin 36, PR.04, line 113
 */
#define GPIO_CHIP "/dev/gpiochip0"
static const unsigned int dir_offsets[PWM_MOTORS] = {106, 113};

static struct gpiod_chip *chip;
static struct gpiod_line *dir_line[PWM_MOTORS];
static int initialized = 0;

static int write_sysfs(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror(path);
        return -errno;
    }
    if (write(fd, value, strlen(value)) < 0) {
        int err = errno;
        fprintf(stderr, "Failed to write %s to %s: %s\n", value, path, strerror(err));
        close(fd);
        return -err;
    }
    close(fd);
    return 0;
}

static void motor_path(char *out, size_t size, int motor, const char *leaf)
{
    snprintf(out, size, "%s/pwm%s/%s", chip_paths[motor], PWM_CHANNEL, leaf);
}

int pwm_init(void)
{
    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        perror("gpiod_chip_open");
        return 1;
    }

    for (int m = 0; m < PWM_MOTORS; m++) {
        dir_line[m] = gpiod_chip_get_line(chip, dir_offsets[m]);
        if (!dir_line[m] || gpiod_line_request_output(dir_line[m], "mdd10-dir", 1) < 0) {
            perror("direction line");
            return 1;
        }
    }

    for (int m = 0; m < PWM_MOTORS; m++) {
        char path[256], export_path[256];
        motor_path(path, sizeof(path), m, "period");
        if (access(path, F_OK) != 0) {
            snprintf(export_path, sizeof(export_path), "%s/export", chip_paths[m]);
            write_sysfs(export_path, PWM_CHANNEL);
            for (int i = 0; i < 50 && access(path, F_OK) != 0; i++)
                usleep(20000);
        }
    }

    for (int m = 0; m < PWM_MOTORS; m++) {
        char path[256];
        motor_path(path, sizeof(path), m, "period");
        if (write_sysfs(path, PWM_PERIOD_STR) < 0)
            return 1;
        motor_path(path, sizeof(path), m, "duty_cycle");
        if (write_sysfs(path, "0") < 0)
            return 1;
        motor_path(path, sizeof(path), m, "enable");
        if (write_sysfs(path, "1") < 0)
            return 1;
    }
    initialized = 1;
    return 0;
}

int pwm_set_duty(int motor, double duty)
{
    if (!initialized || motor < 0 || motor >= PWM_MOTORS)
        return -EINVAL;

    if (duty > PWM_LIMIT)
        duty = PWM_LIMIT;
    else if (duty < -PWM_LIMIT)
        duty = -PWM_LIMIT;

    /* Same polarity as the known-good code: positive -> DIR=1, negative -> DIR=0 */
    if (duty > 0.0)
        gpiod_line_set_value(dir_line[motor], 1);
    else if (duty < 0.0)
        gpiod_line_set_value(dir_line[motor], 0);

    char path[256], value[32];
    motor_path(path, sizeof(path), motor, "duty_cycle");
    snprintf(value, sizeof(value), "%d", (int)(fabs(duty) * PWM_PERIOD_NS));
    return write_sysfs(path, value);
}

void pwm_cleanup(void)
{
    if (!initialized && !chip)
        return;
    for (int m = 0; m < PWM_MOTORS; m++) {
        char path[256];
        motor_path(path, sizeof(path), m, "duty_cycle");
        write_sysfs(path, "0");
    }
    usleep(100000);
    for (int m = 0; m < PWM_MOTORS; m++) {
        char path[256];
        motor_path(path, sizeof(path), m, "enable");
        write_sysfs(path, "0");
        if (dir_line[m]) {
            gpiod_line_release(dir_line[m]);
            dir_line[m] = NULL;
        }
    }
    if (chip) {
        gpiod_chip_close(chip);
        chip = NULL;
    }
    initialized = 0;
}